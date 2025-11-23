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

#define LED_PIN             10
#define I2C_IOEXP_ADDRESS   0x20
#define I2C_OLED_ADDRESS    0x3C
#define I2C_SDA_PIN         4
#define I2C_SCL_PIN         5

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1

/*BUTTONS*/
#define USER_BTN1_PIN 6
#define USER_BTN2_PIN 7
#define USER_BTN3_PIN 8

#define TEMP_AMB_PIN 2
#define TEMP_RES_PIN 3

// allowing a 16-bit write in one transmission.
const byte REG_OUTPUT_PORT_0 = 0x02;

// The desired state: 0x0000 (all 16 pins LOW)
const uint16_t ALL_LOW = 0x0000;

TCA9535 TCA(I2C_IOEXP_ADDRESS);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


void initDisplay();
void displayValue(char* value, int col, int row);
void displayTRES(char* tres);
void displayTAMB(char* tamb);
void getTempAndDisplay();

// Interrupt handler for button 1
void IRAM_ATTR button1ISR() {
    Serial.println("button1");
}

// Interrupt handler for button 2
void IRAM_ATTR button2ISR() {
    Serial.println("button2");
}

// Interrupt handler for button 3
void IRAM_ATTR button3ISR() {
    Serial.println("button3");
}

  // The function that performs the write
void setAllOutputsLow() {
  Serial.print("Sending 0x0000 to Output Port Register (0x02)... ");
  Wire.beginTransmission(I2C_IOEXP_ADDRESS);
  Wire.write(REG_OUTPUT_PORT_0);
  Wire.write((uint8_t)(ALL_LOW & 0xFF));
  Wire.write((uint8_t)(ALL_LOW >> 8));
  byte status = Wire.endTransmission();
  if (status == 0) {
    Serial.println("SUCCESS. Output Latch is now set to LOW.");
  } else {
    Serial.printf("FAILURE. Error code: %d\n", status);
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
    pinMode(USER_BTN1_PIN, INPUT_PULLUP);
    pinMode(USER_BTN2_PIN, INPUT_PULLUP);
    pinMode(USER_BTN3_PIN, INPUT_PULLUP);
    pinMode(TEMP_AMB_PIN, INPUT);
    pinMode(TEMP_RES_PIN, INPUT);
    // Attach interrupts for buttons (trigger on falling edge when pressed)
    attachInterrupt(digitalPinToInterrupt(USER_BTN1_PIN), button1ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(USER_BTN2_PIN), button2ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(USER_BTN3_PIN), button3ISR, FALLING);
    Serial.println("Interrupts attached");
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    TCA.begin();
    if ( !display.begin(SSD1306_SWITCHCAPVCC, I2C_OLED_ADDRESS) ) {
      Serial.println(F("SSD1306 allocation failed"));
      for (;;) { }
    }
    // Execute the critical function immediately after I2C init
    setAllOutputsLow();
    // Initialize display (display.begin is called inside initDisplay)
    initDisplay();
}
void loop() {
    if ( !digitalRead(USER_BTN1_PIN) ) Serial.println("button1");
    if ( !digitalRead(USER_BTN2_PIN) ) Serial.println("button2");
    if ( !digitalRead(USER_BTN3_PIN) ) Serial.println("button3");
    getTempAndDisplay();
    delay(500);
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
    display.fillRect(col, row, 54, 16, SSD1306_BLACK);
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(col, row);
    display.print(value);
    display.display();
}

void displayTRES(char* tres) {
    // initDisplay();
    displayValue(tres, 70, 33);
}
void displayTAMB(char* tamb) {
    // initDisplay();
    displayValue(tamb, 70, 48);
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
