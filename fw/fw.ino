/* Copyright 2025 AHGure
Firmware for step_load project
November 2025
*/

#include <Arduino.h>
#include <Wire.h>
#include "TCA9555.h"

#define LED_PIN 10
#define I2C_ADDRESS 0x20

TCA9535 TCA(I2C_ADDRESS);

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    Serial.begin(9600);
    Wire.begin();
    TCA.begin();

    Wire.setClock(100000);
    TCA.pinMode1(0, OUTPUT);
    TCA.write1(0, HIGH);
}
void loop() {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
}
