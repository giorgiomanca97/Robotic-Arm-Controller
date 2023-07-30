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

#define PIN_TOGGLE 13  // Toggle


// ============================================================
// Parameters
// ============================================================

// Serial Communication
#define BAUDRATE 115200   // Serial baudrate
#define TIMEOUT_US 100000 // Communication Timeout
#define ERROR_MS 1000     // Communication Timeout


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

void setup() {
  toggle.set(true);

  Serial.begin(BAUDRATE);
  Serial.flush();

  timeout.setup(2*TIMEOUT_US);
  Communication::channel(0);
  msg.setCount(6);
  ack.setCount(6);
  err.setCount(6);

  toggle.set(false);

  lcd.begin(16, 2);
  lcd.clear();

  char hex;
  for (uint8_t b = 0; b < 32; b++) {
    if (b == 0) {
      lcd.setCursor(0, 0);
    }
    if (b == 16) {
      lcd.setCursor(0, 1);
    }

    if (b < 16) {
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

void loop() {
  uint32_t time_us = micros();

  toggle.set(true);
  bool res = true;

  msg.setPwm(0, -150);
  msg.setPwm(1, -100);
  msg.setPwm(2, -50);
  msg.setPwm(3, 50);
  msg.setPwm(4, 100);
  msg.setPwm(5, 150);

  if (res) {
    res = Communication::snd(&msg);

    if (!res) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Send");
      lcd.setCursor(0, 1);
      lcd.print("Error");
      delay(ERROR_MS);
    }
  }

  if (res) {
    timeout.reset(time_us);
    res = Communication::peek(&hdr, &timeout);

    if (!res) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Peek");
      lcd.setCursor(0, 1);
      lcd.print("Error");
      delay(ERROR_MS);
    }
  }

  if (res) {
    res = hdr.getCode() == Communication::Code::ACKC;

    if (!res) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Header Code");
      lcd.setCursor(0, 1);
      lcd.print("Error");
      delay(ERROR_MS);
    }
  }

  if (res) {
    res = Communication::rcv(&ack, &timeout);

    if(res) {
      uint8_t buffer[ack.size()];
      ack.fill(buffer);

      char hhex, lhex;
      bool found = false;

      lcd.clear();
      lcd.setCursor(0, 0);

      for(int i = 0; i < ack.size(); i++) {
        if (i == 8) lcd.setCursor(0, 1);
        byteToHex(buffer[i], hhex, lhex);
        lcd.print(hhex);
        lcd.print(lhex);
      }
    } 
    
    if (!res) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Receive");
      lcd.setCursor(0, 1);
      lcd.print("Error");
      delay(ERROR_MS);
    }
  }

  if (!res) {
    Communication::flush();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Communication");
    lcd.setCursor(0, 1);
    lcd.print("Error");
    delay(ERROR_MS);
  }

  delay(TIMEOUT_US/2000);
  toggle.set(false);
  delay(TIMEOUT_US/2000);
}


// ============================================================

#endif