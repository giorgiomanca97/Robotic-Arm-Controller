// Comunication Debugger & Tester

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

#define TOGGLE_COMM 52      // Toggle pin used to check timesampling
#define TOGGLE_CTRL 53      // Toggle pin used to control timesampling
#define TOGGLE_ERROR 51     // Toggle pin used for communication error


// ============================================================
// Parameters
// ============================================================

// Serial Communication
#define COUNT       6       // Motor count
#define CHANNEL     1       // Serial channel
#define BAUDRATE    115200  // Serial baudrate
#define TIMEOUT_US  10000   // Communication Timeout
#define ERROR_MS    1000    // Communication Error wait

// Debug
#if defined(DEBUG_COMMUNICATION)
#define DEBUG_BAUDRATE 115200
#endif


// ============================================================
// Components & Variables
// ============================================================

PinControl toggle_comm = PinControl(TOGGLE_COMM);
PinControl toggle_ctrl = PinControl(TOGGLE_CTRL);
PinControl toggle_error = PinControl(TOGGLE_ERROR);

Timer timer;
Timer timeout;

Communication::Header hdr;
Communication::MsgIDLE msg_idle;
Communication::MsgPWM msg_pwm;
Communication::MsgREF msg_ref;
Communication::MsgROBOT msg_robot;
Communication::MsgMOTOR msg_motor;
Communication::MsgPID msg_pid;
Communication::MsgACKC msg_ackc;
Communication::MsgACKS msg_acks;
Communication::MsgERROR msg_error;

uint8_t choice = 0;
bool state_comm = false;
bool state_ctrl = false;


// ============================================================
// Setup
// ============================================================

void setup() {
  toggle_error.set(true);

  SerialComm::start(CHANNEL, BAUDRATE);

  #if defined(DEBUG_COMMUNICATION)
  SerialComm::start(DEBUG_CHANNEL, DEBUG_BAUDRATE);
  #endif

  timer.setup((uint32_t) 5 * 1000*ERROR_MS);
  timeout.setup((uint32_t) 2*TIMEOUT_US);

  msg_idle.setCount(COUNT);
  msg_pwm.setCount(COUNT);
  msg_ref.setCount(COUNT);
  msg_robot.setCount(COUNT);
  msg_ackc.setCount(COUNT);
  msg_error.setCount(COUNT);

  Communication::channel(CHANNEL);
  Communication::flush();

  delay(5*ERROR_MS);

  toggle_error.set(false);
}


// ============================================================
// Loop
// ============================================================

void loop() {
  uint32_t time_us = micros();
  bool res = true;

  state_comm = !state_comm;
  toggle_comm.set(state_comm);

  if(timer.check(time_us)) {
    res = setup_loop();
    delay(ERROR_MS);
    timer.reset(micros());
  } else {
    state_ctrl = !state_ctrl;
    toggle_ctrl.set(state_ctrl);
    res = ctrl_loop(time_us);
  }

  if(!res) {
    Communication::flush();
    #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
    DEBUG_SERIAL.println("Communication Error");
    #endif
    toggle_error.set(true);
    delay(ERROR_MS);
    toggle_error.set(false);
  }
}


// ============================================================
// Utility
// ============================================================

bool transmit(Communication::Message *sndMsg, Communication::Message *rcvMsg, Timer *timeout);
bool transmit(Communication::Message *sndMsg, Communication::Message *rcvMsg, Timer *timeout) {
  Communication::Header hdr;

  if (!Communication::snd(sndMsg)) {
    #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
    DEBUG_SERIAL.println("Send Error");
    #endif
    return false;
  }

  if (!Communication::peek(&hdr, rcvMsg->getCode(), timeout)) {
    #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
    DEBUG_SERIAL.println("Peek Error");
    #endif
    return false;
  }

  if (hdr.getCode() != rcvMsg->getCode()) {
    #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
    DEBUG_SERIAL.print("Code Error (");
    DEBUG_SERIAL.print((uint8_t) hdr.getCode());
    DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print((uint8_t) rcvMsg->getCode());
    DEBUG_SERIAL.println(")");
    #endif
    return false;
  }

  if (!Communication::rcv(rcvMsg, timeout)) {
    #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
    DEBUG_SERIAL.println("Receive Error");
    #endif
    return false;
  }

  return true;
}


bool setup_loop() {
  msg_robot.setTimeSampling(TIMEOUT_US);

  #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
  DEBUG_SERIAL.println("Transmitting: Setup ROBOT -> Ack ACKS");
  #endif

  timeout.reset(micros());
  if(!transmit(&msg_robot, &msg_acks, &timeout)) {
    #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
    DEBUG_SERIAL.println("  operation: Failed");
    #if defined(DEBUG_DATA)
    DebugComm::print(&msg_robot, "", 1);
    #endif
    #endif
    return false;
  }

  #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
  DEBUG_SERIAL.println("  operation: Succeded");
  #if defined(DEBUG_DATA)
  DebugComm::print(&msg_robot, "", 1);
  DebugComm::print(&msg_acks, "", 1);
  #endif
  #endif

  for(int i = 0; i < COUNT; i++) {
    msg_motor.setIndex(i);
    msg_motor.setEncDirection(1);
    msg_motor.setSpinDirection(-1);
    msg_motor.setChangeEncoder(true);
    msg_motor.setEncoderValue((int32_t) i * 10000);

    #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
    DEBUG_SERIAL.println("Transmitting: Setup MOTOR -> Ack ACKS");
    #endif

    timeout.reset(micros());
    if(!transmit(&msg_motor, &msg_acks, &timeout)) {
      #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
      DEBUG_SERIAL.println("  operation: Failed");
      #if defined(DEBUG_DATA)
      DebugComm::print(&msg_motor, "", 1);
      #endif
      #endif
      return false;
    }

    #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
    DEBUG_SERIAL.println("  operation: Succeded");
    #if defined(DEBUG_DATA)
    DebugComm::print(&msg_motor, "", 1);
    DebugComm::print(&msg_acks, "", 1);
    #endif
    #endif
  }

  for(int i = 0; i < COUNT; i++) {
    msg_pid.setIndex(i);
    msg_pid.setPidDiv(i * 1000.0f);
    msg_pid.setPidKp(i * 1000.0f);
    msg_pid.setPidKi(i * 100.0f);
    msg_pid.setPidKd(i * 10.0f);
    msg_pid.setPidSat(i * 10000.0f);
    msg_pid.setPidPole(i * 1.0f);

    #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
    DEBUG_SERIAL.println("Transmitting: Setup PID -> Ack ACKS");
    #endif

    timeout.reset(micros());
    if(!transmit(&msg_pid, &msg_acks, &timeout)) {
      #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
      DEBUG_SERIAL.println("  operation: Failed");
      #if defined(DEBUG_DATA)
      DebugComm::print(&msg_pid, "", 1);
      #endif
      #endif
      return false;
    }

    #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
    DEBUG_SERIAL.println("  operation: Succeded");
    #if defined(DEBUG_DATA)
    DebugComm::print(&msg_pid, "", 1);
    DebugComm::print(&msg_acks, "", 1);
    #endif
    #endif
  }

  choice = (choice+1) % 3;
  return true;
}


bool ctrl_loop(uint32_t time_us) {
  switch(choice) {
    case 0:
      #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
      DEBUG_SERIAL.println("Transmitting: Control IDLE -> Ack ACKC");
      #endif

      timeout.reset(time_us);
      if(!transmit(&msg_idle, &msg_ackc, &timeout)) {
        #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
        DEBUG_SERIAL.println("  operation: Failed");
        #if defined(DEBUG_DATA)
        DebugComm::print(&msg_idle, "", 1);
        #endif
        #endif
        return false;
      }

      #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
      DEBUG_SERIAL.println("  operation: Succeded");
      #if defined(DEBUG_DATA)
      DebugComm::print(&msg_idle, "", 1);
      DebugComm::print(&msg_ackc, "", 1);
      #endif
      #endif
      return true;

    case 1:
      msg_pwm.setPwm(0, -150);
      msg_pwm.setPwm(1, -100);
      msg_pwm.setPwm(2, -50);
      msg_pwm.setPwm(3, 50);
      msg_pwm.setPwm(4, 100);
      msg_pwm.setPwm(5, 150);

      #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
      DEBUG_SERIAL.println("Transmitting: Control PWM -> Ack ACKC");
      #endif

      timeout.reset(time_us);
      if(!transmit(&msg_pwm, &msg_ackc, &timeout)) {
        #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
        DEBUG_SERIAL.println("  operation: Failed");
        #if defined(DEBUG_DATA)
        DebugComm::print(&msg_pwm, "", 1);
        #endif
        #endif
        return false;
      }

      #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
      DEBUG_SERIAL.println("  operation: Succeded");
      #if defined(DEBUG_DATA)
      DebugComm::print(&msg_pwm, "", 1);
      DebugComm::print(&msg_ackc, "", 1);
      #endif
      #endif
      return true;

    case 2:
      msg_ref.setDeltaEnc(0, +10);
      msg_ref.setDeltaEnc(1, +20);
      msg_ref.setDeltaEnc(2, +50);
      msg_ref.setDeltaEnc(3, -15);
      msg_ref.setDeltaEnc(4, -30);
      msg_ref.setDeltaEnc(5, -60);

      #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
      DEBUG_SERIAL.println("Transmitting: Control REF -> Ack ACKC");
      #endif

      timeout.reset(time_us);
      if(!transmit(&msg_ref, &msg_ackc, &timeout)) {
        #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
        DEBUG_SERIAL.println("  operation: Failed");
        #if defined(DEBUG_DATA)
        DebugComm::print(&msg_ref, "", 1);
        #endif
        #endif
        return false;
      }

      #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
      DEBUG_SERIAL.println("  operation: Succeded");
      #if defined(DEBUG_DATA)
      DebugComm::print(&msg_ref, "", 1);
      DebugComm::print(&msg_ackc, "", 1);
      #endif
      #endif
      return true;

    default:
      return false;
  }
}


// ============================================================

#endif