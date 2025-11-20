/* Copyright 2025 AHGure
Firmware for step_load project
November 2025
*/

#include <Arduino.h>
#include <Wire.h>
#include "TCA9555.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define LED_PIN 10
#define I2C_IOEXP_ADDRESS 0x20
#define I2C_OLED_ADDRESS 0x3C
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 32    // OLED display height, in pixels
#define OLED_RESET     -1   // Reset pin # (or -1 if sharing Arduino reset pin)

// allowing a 16-bit write in one transmission.
const byte REG_OUTPUT_PORT_0 = 0x02; 

// The desired state: 0x0000 (all 16 pins LOW)
const uint16_t ALL_LOW = 0x0000;

TCA9535 TCA(I2C_IOEXP_ADDRESS);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10    // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };


  // The function that performs the write
void setAllOutputsLow() {
  Serial.print("Sending 0x0000 to Output Port Register (0x02)... ");
  
  // Start the I2C transmission to the TCA9535
  Wire.beginTransmission(I2C_IOEXP_ADDRESS);
  
  // 1. Specify the register we want to write to (Output Port 0)
  Wire.write(REG_OUTPUT_PORT_0); 
  
  // 2. Write the LOW byte (Port 0: P0_0 to P0_7)
  // We use bitwise shifting to extract the low 8 bits of ALL_LOW (which is 0x0000)
  Wire.write((uint8_t)(ALL_LOW & 0xFF)); 
  
  // 3. Write the HIGH byte (Port 1: P1_0 to P1_7)
  // We use bitwise shifting to extract the high 8 bits of ALL_LOW (which is 0x0000)
  Wire.write((uint8_t)(ALL_LOW >> 8)); 
  
  // End the transmission and check for success
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
    Serial.begin(9600);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    
    TCA.begin();
    // Execute the critical function immediately after I2C init
    setAllOutputsLow(); 

    if ( !display.begin(SSD1306_SWITCHCAPVCC, I2C_OLED_ADDRESS) ) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;) {}
    }
    // Clear the buffer
    display.clearDisplay();
    display.invertDisplay(true);
    delay(1000);
    display.invertDisplay(false);
    delay(1000);
    for ( int i = 0; i < 16; i++ ) {
        TCA.pinMode1(i, OUTPUT);
        TCA.write1(i, LOW);
    }
    delay(2000);
    for ( int i = 0; i < 16; i++ ) {
        TCA.write1(i, HIGH);
        delay(3000);
        TCA.write1(i, LOW);
        delay(10);
    }


    // TCA.write1(0, HIGH);
    // delay(5000);
    // TCA.write1(1, HIGH);
    // delay(5000);
    // TCA.write1(2, HIGH);
    // delay(5000);
    // TCA.write1(3, HIGH);
    // delay(5000);
    // display.display();
    // delay(2000);    // Pause for 2 seconds
}
void loop() {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
}
