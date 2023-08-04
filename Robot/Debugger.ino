// Debug code

#if SELECT_SKETCH == 2


// ============================================================
// Includes
// ============================================================

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
#define CHANNEL     1       // Serial channel
#define BAUDRATE    115200  // Serial baudrate
#define TIMEOUT_US  100000  // Communication Timeout
#define ERROR_MS    1000    // Communication Timeout

// Debug
#if defined(DEBUG_COMMUNICATION)
#define DEBUG_SERIAL_ENABLE
#define DEBUG_SERIAL_BAUDRATE 115200
#endif


// ============================================================
// Components & Variables
// ============================================================

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

  switch(CHANNEL) {
    case 0:
      Serial.begin(BAUDRATE);
      Serial.flush();
    case 1:
      Serial1.begin(BAUDRATE);
      Serial1.flush();
    case 2:
      Serial2.begin(BAUDRATE);
      Serial2.flush();
    case 3:
      Serial3.begin(BAUDRATE);
      Serial3.flush();
  }

  #if defined(DEBUG_SERIAL_ENABLE)
    Serial.begin(DEBUG_SERIAL_BAUDRATE);
    Serial.flush();
  #endif

  timeout.setup(2*TIMEOUT_US);

  Communication::channel(CHANNEL);
  msg.setCount(6);
  ack.setCount(6);
  err.setCount(6);

  toggle.set(false);
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
      Serial.println("Send Error");
      delay(ERROR_MS);
    }
  }

  if (res) {
    timeout.reset(time_us);
    res = Communication::peek(&hdr, &timeout);

    if (!res) {
      Serial.println("Peek Error");
      delay(ERROR_MS);
    }
  }

  if (res) {
    res = hdr.getCode() == Communication::Code::ACKC;

    if (!res) {
      Serial.print("Code (");
      Serial.print((uint8_t) hdr.getCode());
      Serial.println(") Error");
      delay(ERROR_MS);
    }
  }

  if (res) {
    res = Communication::rcv(&ack, &timeout);
    
    if (!res) {
      Serial.println("Receive Error");
      delay(ERROR_MS);
    }
  }

  if (!res) {
    Communication::flush();
    Serial.println("Communication Error");
    delay(ERROR_MS);
  }

  toggle.set(false);
}


// ============================================================

#endif