/* Copyright 2025 AHGure
Firmware for step_load project
November 2025
*/

#include <Arduino.h>
#include <Wire.h>
#include "TCA9555.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <cstdio>

// UART COMMUNICATION
#define SYNC_BYTE       0x51
#define READ_CMD        0x01
#define WRTITE_CMD      0x02

#define LED_PIN             10
#define I2C_IOEXP_ADDRESS   0x20
#define I2C_OLED_ADDRESS    0x3C
#define I2C_SDA_PIN         4
#define I2C_SCL_PIN         5

#define SCREEN_WIDTH        128
#define SCREEN_HEIGHT       64
#define OLED_RESET          -1

/*BUTTONS*/
#define USER_BTN1_PIN   6
#define USER_BTN2_PIN   7
#define USER_BTN3_PIN   8

#define TEMP_AMB_PIN    2
#define TEMP_RES_PIN    3

#define DEBOUNCE_MS     200

// allowing a 16-bit write in one transmission.
const byte REG_OUTPUT_PORT_0 = 0x02;

// The desired state: 0x0000 (all 16 pins LOW)
const uint16_t ALL_LOW = 0x0000;

volatile int btn1Pressed = 0;
volatile int btn2Pressed = 0;
volatile int btn3Pressed = 0;

volatile uint64_t btn1PressTimer = 0;
volatile uint64_t btn2PressTimer = 0;
volatile uint64_t btn3PressTimer = 0;

volatile uint8_t selectedResistor = 0;
volatile uint8_t previousResistor = 0;

enum changeState {
    NO_CHANGE,
    INCREASE,
    DECREASE,
    ACCEPT_CHANGE
} ResistorState = NO_CHANGE;

enum CommandState {
    IDLE,
    PARALLEL_RESISTOR,
    CONFIRM_RESISTOR,
} CmdState = IDLE;

struct uart {
    uint8_t sync;
    uint8_t command;
    uint8_t data[4];
};
uart uartPacket;


TCA9535 TCA(I2C_IOEXP_ADDRESS);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

volatile uint64_t lastBtn1Interrupt = 0;
volatile uint64_t lastBtn2Interrupt = 0;
volatile uint64_t lastBtn3Interrupt = 0;

volatile uint8_t RXData[10];
volatile bool dataReceived = false;
volatile uint8_t rxIndex = 0;

void IRAM_ATTR handleBtnPress() {
  noInterrupts();  // Disable interrupts during processing
  uint64_t currentTime = millis();

  if ( digitalRead(USER_BTN1_PIN) == LOW ) {
      if (currentTime - lastBtn1Interrupt > DEBOUNCE_MS) {
        btn1Pressed = 1;
        lastBtn1Interrupt = currentTime;
      }
  } else if ( digitalRead(USER_BTN2_PIN) == LOW ) {
      if (currentTime - lastBtn2Interrupt > DEBOUNCE_MS) {
        btn2Pressed = 1;
        lastBtn2Interrupt = currentTime;
      }
  } else if ( digitalRead(USER_BTN3_PIN) == LOW ) {
      if (currentTime - lastBtn3Interrupt > DEBOUNCE_MS) {
        btn3Pressed = 1;
        lastBtn3Interrupt = currentTime;
      }
  }
}

void readSerial() {
    static uint8_t buffer[10];
    static uint8_t index = 0;
    static unsigned long lastByteTime = 0;
    static unsigned long debugTimer = 0;
    unsigned long currentTime = millis();

    // Debug: Print every 5 seconds if waiting for data
    if (currentTime - debugTimer > 5000) {
        Serial.print("Waiting for serial data. Available: ");
        Serial.println(Serial.available());
        debugTimer = currentTime;
    }

    // Reset buffer if timeout (500ms since last byte)
    if (index > 0 && (currentTime - lastByteTime) > 500) {
        Serial.println("Buffer timeout - resetting");
        index = 0;
    }

    while (Serial.available() > 0) {
        uint8_t inByte = Serial.read();
        lastByteTime = currentTime;

        Serial.print("Received byte[");
        Serial.print(index);
        Serial.print("]: 0x");
        if (inByte < 16) Serial.print("0");
        Serial.println(inByte, HEX);

        // If we see sync byte, start fresh
        if (inByte == SYNC_BYTE && index != 0) {
            Serial.println("SYNC byte detected - restarting buffer");
            index = 0;
        }

        buffer[index++] = inByte;

        // If buffer is full, process it
        if (index >= 10) {
            Serial.println("Buffer full - processing");
            if (buffer[0] == SYNC_BYTE) {
                Serial.println("Valid sync byte - copying to RXData");
                noInterrupts();
                for (uint8_t i = 0; i < 10; i++) {
                    RXData[i] = buffer[i];
                }
                dataReceived = true;
                interrupts();
            } else {
                Serial.print("Invalid sync byte: 0x");
                Serial.println(buffer[0], HEX);
            }
            index = 0;
        }
    }
}

void initDisplay();
void displayValue(char* value, int col, int row);
void displayTRES(char* tres);
void displayTAMB(char* tamb);
void displayResistor(char* resistor);
void getTempAndDisplay();
void initIOExpander();
uint8_t selectResistor(uint8_t parResistor);
void setResistorAndDelay(uint8_t parResistor);


  // The function that performs the write
void setAllOutputsLow() {
  Wire.beginTransmission(I2C_IOEXP_ADDRESS);
  Wire.write(REG_OUTPUT_PORT_0);
  Wire.write((uint8_t)(ALL_LOW & 0xFF));
  Wire.write((uint8_t)(ALL_LOW >> 8));
  byte status = Wire.endTransmission();
  if (status == 0) {
    Serial.println("IO Expander: All outputs set to LOW");
  } else {
    Serial.printf("IO Expander error: %d\n", status);
  }
}

void LEDBlink() {
    for (int i = 0; i < 2; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(50);
        digitalWrite(LED_PIN, LOW);
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    Serial.begin(115200);
    // Wait for serial to initialize
    delay(100);
    Serial.println("Starting setup...");
    // Configure button pins with internal pull-up resistors
    pinMode(USER_BTN1_PIN, INPUT);
    pinMode(USER_BTN2_PIN, INPUT);
    pinMode(USER_BTN3_PIN, INPUT);
    pinMode(TEMP_AMB_PIN, INPUT);
    pinMode(TEMP_RES_PIN, INPUT);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    TCA.begin();
    if ( !display.begin(SSD1306_SWITCHCAPVCC, I2C_OLED_ADDRESS) ) {
      Serial.println(F("SSD1306 connection failed!"));
      for (;;) { }
    }
    // Execute the critical function immediately after I2C init
    setAllOutputsLow();
    initIOExpander();
    // Initialize display (display.begin is called inside initDisplay)
    initDisplay();
    // Display initial state (all relays off)
    setResistorAndDelay(selectedResistor);

    attachInterrupt(digitalPinToInterrupt(USER_BTN1_PIN),
                    handleBtnPress, FALLING);
    attachInterrupt(digitalPinToInterrupt(USER_BTN2_PIN),
                    handleBtnPress, FALLING);
    attachInterrupt(digitalPinToInterrupt(USER_BTN3_PIN),
                    handleBtnPress, FALLING);
}

void loop() {
    static int lastBtn1State = -1;
    static int lastBtn2State = -1;
    static int lastBtn3State = -1;

    // Check for serial data
    readSerial();

    if (dataReceived) {
      noInterrupts();
      uint8_t localData[10];
      for (uint8_t i = 0; i < 10; i++) {
          localData[i] = RXData[i];
      }
      dataReceived = false;
      interrupts();

      Serial.println("=== Serial Data Received ===");
      for (uint8_t i = 0; i < 10; i++) {
          Serial.print("RXData[");
          Serial.print(i);
          Serial.print("]: 0x");
          if (localData[i] < 16) Serial.print("0");
          Serial.println(localData[i], HEX);
      }
      Serial.println("===========================");
    }
    // Handle Button 1 Press (only if state is NO_CHANGE)
    if (btn1Pressed && ResistorState == NO_CHANGE) {
      btn1Pressed = 0;  // Clear flag immediately
      ResistorState = DECREASE;
      LEDBlink();
    }
    // Handle Button 2 Press (only if state is NO_CHANGE)
    if (btn2Pressed && ResistorState == NO_CHANGE) {
        btn2Pressed = 0;  // Clear flag immediately
        ResistorState = INCREASE;
        LEDBlink();
    }
    // Handle Button 3 Press
    if (btn3Pressed) {
        btn3Pressed = 0;  // Clear flag immediately
        ResistorState = ACCEPT_CHANGE;
        LEDBlink();
    }
    switch (ResistorState) {
      case INCREASE:
        Serial.println("ADD RESISTOR");
        if (selectedResistor < 16) selectedResistor++;
        else selectedResistor = 16;
        ResistorState = NO_CHANGE;
        break;
      case DECREASE:
        Serial.println("REMOVE RESISTOR");
        if (selectedResistor < 1) selectedResistor=0;
        else selectedResistor--;
        ResistorState = NO_CHANGE;
        break;
      case ACCEPT_CHANGE:
        Serial.println("ACCEPT_CHANGE");
        // Currently no specific action defined for ACCEPT_CHANGE
        Serial.print("No. Resistors in parallel: ");
        Serial.println(selectedResistor);
        setResistorAndDelay(selectedResistor);
        ResistorState = NO_CHANGE;
        break;
      case NO_CHANGE:
        break;
    }

    getTempAndDisplay();
    // setResistorAndDelay(1);  // Removed: was resetting to 1 every loop
    delay(500);
    interrupts();
}

void initDisplay() {
  display.clearDisplay();

  // Draw shapes
  display.drawRect(0, 0, 127, 64, SSD1306_WHITE);
  display.drawLine(0, 17, 126, 17, SSD1306_WHITE);
  // Draw text labels
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(4, 2);
  display.print(F("LOAD STEP"));
  // Resistor Label
  display.setTextSize(2);
  display.setCursor(4, 18);
  display.print(F("RES"));
  display.setCursor(70, 18);
  display.print(F("6.25"));
  // Resistor Temperature Label
  display.setCursor(4, 33);
  display.print(F("T.RES"));
  // Ambient Temperature Label
  display.setCursor(4, 48);
  display.print(F("T.AMB"));
  // Vertical line
  display.drawLine(64, 17, 64, 64, SSD1306_WHITE);
  // Initial values
  display.display();
}

void displayValue(char* value, int col, int row) {
    // initDisplay();
    // Clear the area before displaying new value
    display.fillRect(col, row, 58, 17, SSD1306_BLACK);
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);  // Disable text wrapping
    display.setCursor(col, row);
    display.print(value);
    display.display();
}

void displayTRES(char* tres)
{
    // initDisplay();
    displayValue(tres, 68, 33);
}

void displayTAMB(char* tamb) {
    // initDisplay();
    displayValue(tamb, 68, 48);
}

void displayResistor(char* resistor) {
    // initDisplay();
    displayValue(resistor, 68, 18);
}

void getTempAndDisplay() {
    // Read LM35DZ sensor value
    int tempResistor = analogRead(TEMP_RES_PIN);
    int tempAmbient = analogRead(TEMP_AMB_PIN);
    float temperatureC = (tempResistor * 3.3 * 100.0) / 4096.0;
    float temperatureAmbC = (tempAmbient * 3.3 * 100.0) / 4096.0;
    char temperatureCStr[6];
    char temperatureAmbCStr[6];
    snprintf((char*)temperatureCStr, sizeof(temperatureCStr),
            "%.1f", temperatureC);
    snprintf((char*)temperatureAmbCStr, sizeof(temperatureAmbCStr),
            "%.1f", temperatureAmbC);
    displayTRES(temperatureCStr);
    displayTAMB(temperatureAmbCStr);
}

void initIOExpander() {
    // TCA.begin();
    // Set all pins of Port 0 and Port 1 as outputs (0)
    TCA.pinMode1(0, OUTPUT);
    TCA.pinMode1(1, OUTPUT);
    TCA.pinMode1(2, OUTPUT);
    TCA.pinMode1(3, OUTPUT);
    TCA.pinMode1(4, OUTPUT);
    TCA.pinMode1(5, OUTPUT);
    TCA.pinMode1(6, OUTPUT);
    TCA.pinMode1(7, OUTPUT);
    TCA.pinMode1(8, OUTPUT);
    TCA.pinMode1(9, OUTPUT);
    TCA.pinMode1(10, OUTPUT);
    TCA.pinMode1(11, OUTPUT);
    TCA.pinMode1(12, OUTPUT);
    TCA.pinMode1(13, OUTPUT);
    TCA.pinMode1(14, OUTPUT);
    TCA.pinMode1(15, OUTPUT);
}

uint8_t selectResistor(uint8_t parResistor) {
    uint8_t currentResistor = parResistor;
    int delta = currentResistor - previousResistor;

    if (parResistor > 16) parResistor = 16;
    else if (parResistor < 0) parResistor = 0;

    if ( parResistor == 0 ) {
      // Set all pins LOW (all relays off)
      for (uint8_t i = 0; i < 16; i++) {
        TCA.write1(i, LOW);
      }
    } else {
      if (delta > 0) {
        TCA.write1(currentResistor-1, HIGH);
      } else if (delta < 0) {
        TCA.write1(previousResistor-1, LOW);
      }
    }

    previousResistor = currentResistor;
    return parResistor;
}

void setResistorAndDelay(uint8_t parResistor) {
  char parResistorStr[6];
  if (parResistor > 16) {
    parResistor = 16;
  } else if (parResistor < 1) {
    parResistor = 0;
  }
  if (parResistor == 0) {
    snprintf((char*)parResistorStr, sizeof(parResistorStr),
            "%s", "INF");
  } else {
    snprintf((char*)parResistorStr, sizeof(parResistorStr),
            "%.2f", 100.0 / parResistor);
  }
  selectResistor(parResistor);
  displayResistor(parResistorStr);
  delay(10);
}
