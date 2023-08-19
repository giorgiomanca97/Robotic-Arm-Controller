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

#define PIN_TOGGLE 52       // Toggle pin used to check timesampling


// ============================================================
// Parameters
// ============================================================

// Serial Communication
#define COUNT       6       // Motor count
#define CHANNEL     1       // Serial channel
#define BAUDRATE    115200  // Serial baudrate
#define TIMEOUT_US  250000  // Communication Timeout
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
Timer timer;
Timer timeout;

Communication::Header hdr;
Communication::MsgIDLE msgIDLE;
Communication::MsgPWM msgPWM;
Communication::MsgREF msgREF;
Communication::MsgROBOT msgROBOT;
Communication::MsgMOTOR msgMOTOR;
Communication::MsgPID msgPID;
Communication::MsgACKC msgACKC;
Communication::MsgACKS msgACKS;
Communication::MsgERROR msgERROR;

uint8_t choice = 0;


// ============================================================
// Utility
// ============================================================

bool transmit(Communication::Message *sndMsg, Communication::Message *rcvMsg, Timer *timeout);
bool transmit(Communication::Message *sndMsg, Communication::Message *rcvMsg, Timer *timeout) {
  Communication::Header hdr;

  if (!Communication::snd(sndMsg)) {
    Serial.println("Send Error");
    return false;
  }

  if (!Communication::peek(&hdr, timeout)) {
    Serial.println("Peek Error");
    return false;
  }

  if (hdr.getCode() != rcvMsg->getCode()) {
    Serial.print("Code (");
    Serial.print((uint8_t) hdr.getCode());
    Serial.println(") Error");
    return false;
  }
  
  if (!Communication::rcv(rcvMsg, timeout)) {
    Serial.println("Receive Error");
    return false;
  }

  return true;
}


bool setup_loop() {
  msgROBOT.setTimeSampling(TIMEOUT_US);

  timeout.reset(micros());
  if(!transmit(&msgROBOT, &msgACKS, &timeout)) {
    return false;
  }

  for(int i = 0; i < COUNT; i++) {
    msgMOTOR.setIndex(i);
    msgMOTOR.setEncDirection(1);
    msgMOTOR.setSpinDirection(-1);
    msgMOTOR.setChangeEncoder(true);
    msgMOTOR.setEncoderValue((int32_t) i * 10000);
    
    timeout.reset(micros());
    if(!transmit(&msgMOTOR, &msgACKS, &timeout)) {
      return false;
    }
  }

  for(int i = 0; i < COUNT; i++) {
    msgPID.setIndex(i);
    msgPID.setPidDiv(i * 1000.0f);
    msgPID.setPidKp(i * 1000.0f);
    msgPID.setPidKi(i * 100.0f);
    msgPID.setPidKd(i * 10.0f);
    msgPID.setPidSat(i * 10000.0f);
    msgPID.setPidPole(i * 1.0f);

    timeout.reset(micros());
    if(!transmit(&msgPID, &msgACKS, &timeout)) {
      return false;
    }
  }

  choice = (choice+1) % 3;
  return true;
}


bool ctrl_loop(uint32_t time_us) {
  timeout.reset(time_us);

  switch(choice) {
    case 0:
      return transmit(&msgIDLE, &msgACKC, &timeout);

    case 1:
      msgPWM.setPwm(0, -150);
      msgPWM.setPwm(1, -100);
      msgPWM.setPwm(2, -50);
      msgPWM.setPwm(3, 50);
      msgPWM.setPwm(4, 100);
      msgPWM.setPwm(5, 150);
      return transmit(&msgPWM, &msgACKC, &timeout);

    case 2:
      return transmit(&msgREF, &msgACKC, &timeout);

    default:
      return false;
  }
}


// ============================================================
// Setup
// ============================================================

void setup() {
  toggle.set(true);

  switch(CHANNEL) {
    case 0:
      Serial.begin(BAUDRATE);
      Serial.flush();
      break;

    case 1:
      Serial1.begin(BAUDRATE);
      Serial1.flush();
      break;

    case 2:
      Serial2.begin(BAUDRATE);
      Serial2.flush();
      break;

    case 3:
      Serial3.begin(BAUDRATE);
      Serial3.flush();
      break;
  }

  #if defined(DEBUG_SERIAL_ENABLE)
    Serial.begin(DEBUG_SERIAL_BAUDRATE);
    Serial.flush();
  #endif

  timer.setup(20*TIMEOUT_US);
  timeout.setup(2*TIMEOUT_US);

  Communication::channel(CHANNEL);
  msgIDLE.setCount(COUNT);
  msgPWM.setCount(COUNT);
  msgREF.setCount(COUNT);
  msgROBOT.setCount(COUNT);
  msgACKC.setCount(COUNT);
  msgERROR.setCount(COUNT);

  delay(5*ERROR_MS);

  toggle.set(false);
}


// ============================================================
// Loop
// ============================================================

void loop() {
  uint32_t time_us = micros();
  bool res = true;

  if(timer.check(time_us)) 
  {
    res = setup_loop();
    delay(ERROR_MS);
    timer.reset(micros());
  } else { 
    toggle.set(true);
    res = ctrl_loop(time_us);
    toggle.set(false);
  }

  if(!res) {
    Communication::flush();
    Serial.println("Communication Error");
    delay(ERROR_MS);
  }
}


// ============================================================

#endif