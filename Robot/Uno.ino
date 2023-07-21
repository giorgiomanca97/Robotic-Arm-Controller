// Debug code

#if defined(UNO)


// ============================================================
// Includes
// ============================================================

#include <LiquidCrystal.h>

#include "utils.h"
#include "control.h"
#include "components.h"
#include "communication.h"


// ============================================================
// Pins
// ============================================================

#define PIN_TOGGLE  13      // Toggle


// ============================================================
// Parameters
// ============================================================

// Serial Communication
#define BAUDRATE    115200  // Serial baudrate
#define TIMEOUT_US   10000  // Communication Timeout
#define ERROR_MS      1000  // Communication Timeout


// ============================================================
// Components & Variables
// ============================================================

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

PinControl toggle = PinControl(PIN_TOGGLE);
Timer timeout;

Communication::MsgPWM msg;
Communication::MsgACKC ack;
Communication::MsgERROR err;
Communication::Header hdr;


// ============================================================
// Setup
// ============================================================

void setup()
{
  toggle.set(true);

  Serial.begin(BAUDRATE);
  Serial.flush();

  timeout.setup(TIMEOUT_US);
  Communication::channel(0);
  msg.setCount(6);
  ack.setCount(6);
  err.setCount(6);

  toggle.set(false);

  lcd.begin(16, 2);
  lcd.clear();
  
  char hex;
  for(uint8_t b = 0; b < 32; b++) {
    if(b == 0) {
      lcd.setCursor(0, 0);
    }
    if(b == 16) {
      lcd.setCursor(0, 1);
    }

    if(b < 16) {
      nibbleToHex(b, hex);
    } else {
      nibbleToHex(b - 16, hex);
    }

    lcd.print(hex);
  }

  delay(ERROR_MS);
  lcd.clear();
}


// ============================================================
// Loop
// ============================================================

void loop()
{
  uint32_t time_us = micros();
  bool res = true;

  if(Serial.available()) {
    toggle.set(true);

    char hhex, lhex;
    bool found = false;

    lcd.clear();
    lcd.setCursor(0, 0);

    int i = 0;
    while(Serial.available() && i < 16) {
      if(i == 8) lcd.setCursor(0, 1);
      i++;
      uint8_t byte = Serial.read();
      byteToHex(byte, hhex, lhex);
      lcd.print(hhex);
      lcd.print(lhex);
    }

    toggle.set(false);
  }
  

  msg.setPwm(0, -150);
  msg.setPwm(1, -100);
  msg.setPwm(2,  -50);
  msg.setPwm(3,   50);
  msg.setPwm(4,  100);
  msg.setPwm(5,  150);

  if(res) {
    res = Communication::snd(&msg);
  }

  if(res) {
    timeout.reset(time_us);
    res = Communication::peek(&hdr, &timeout);
  }
  
  if(!res) {
    toggle.set(true);
    delay(ERROR_MS/2);
    toggle.set(false);
    delay(ERROR_MS/2);
  }
}


// ============================================================
// Utility
// ============================================================

void byteToHex(const uint8_t & byte, char & hhex, char & lhex) {
  nibbleToHex((byte & 0b00001111) >> 0, lhex);
  nibbleToHex((byte & 0b11110000) >> 4, hhex);
}

void nibbleToHex(const uint8_t & nibble, char & hex) {
  switch(nibble) {
    case 0u:
      hex = '0';
      break;
    case 1u:
      hex = '1';
      break;
    case 2u:
      hex = '2';
      break;
    case 3u:
      hex = '3';
      break;
    case 4u:
      hex = '4';
      break;
    case 5u:
      hex = '5';
      break;
    case 6u:
      hex = '6';
      break;
    case 7u:
      hex = '7';
      break;
    case 8u:
      hex = '8';
      break;
    case 9u:
      hex = '9';
      break;
    case 10u:
      hex = 'A';
      break;
    case 11u:
      hex = 'B';
      break;
    case 12u:
      hex = 'C';
      break;
    case 13u:
      hex = 'D';
      break;
    case 14u:
      hex = 'E';
      break;
    case 15u:
      hex = 'F';
      break;
  }
}

// ============================================================

#endif