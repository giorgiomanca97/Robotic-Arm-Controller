#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define MEGA
#endif

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
#define UNO
#endif

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
#define LEONARDO
#endif


#include <Arduino.h>
#include <math.h>
#include "utils.h"
#include "components.h"


// Serial Communication Protocol (static)
class Communication {
  inline static HardwareSerial *hwserial = &Serial;

public:
  Communication() = delete;
  ~Communication() = delete;
  
  static void channel(uint8_t index);
  static void flush();
  
  // Codes
  enum class Code : uint8_t{
    IDLE  = 0b00000,   // 0
    PWM   = 0b00001,   // 1
    REF   = 0b00010,   // 2
    ROBOT = 0b10000,   // 16
    MOTOR = 0b10001,   // 17
    PID   = 0b10010,   // 18
    ACKC  = 0b11000,   // 24
    ACKS  = 0b11001,   // 25
    ERROR = 0b11111    // 31
  };

  static bool convert(uint8_t value, Code &code);
  static bool isCtrl(Code code);
  static bool isSetup(Code code);
  static bool isAck(Code code);
  static bool isError(Code code);

  struct Header{
    Header(Code code, uint8_t num);
    Header() : Header(Code::IDLE, 0) {}

    Code getCode();
    uint8_t getNum();

    bool setCode(Code code);
    bool setNum(uint8_t num);

    bool parse(uint8_t byte);
    uint8_t byte();

    uint8_t size();
    uint8_t from(uint8_t *buffer);
    uint8_t fill(uint8_t *buffer);

  private:
    Code code;
    uint8_t num;
  };

  
  struct Message{
    Message(Code code, uint8_t num);
    Message(Code code) : Message(code, 0) {}

    Code getCode();
    uint8_t getNum();

    bool setNum(uint8_t num);

    uint8_t size();
    uint8_t from(uint8_t *buffer);
    uint8_t fill(uint8_t *buffer);

  protected:
    virtual uint8_t size_payload();
    virtual uint8_t from_payload(uint8_t *buffer);
    virtual uint8_t fill_payload(uint8_t *buffer);

  private:
    Header header;

    uint8_t size_header();
    uint8_t from_header(uint8_t *buffer);
    uint8_t fill_header(uint8_t *buffer);
  };


  struct MsgIDLE : public Message{
    MsgIDLE() : Message(Code::IDLE) {}
    MsgIDLE(uint8_t num) : Message(Code::IDLE, num) {}

    uint8_t getCount();
    
    bool setCount(uint8_t count);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);
  };


  struct MsgPWM : public Message{
    MsgPWM() : Message(Code::PWM) {}
    MsgPWM(uint8_t num) : Message(Code::PWM, num) {}

    uint8_t getCount();
    int16_t getPwm(uint8_t index);

    bool setCount(uint8_t count);
    bool setPwm(uint8_t index, int16_t value);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);
    
  private:
    int16_t pwms[8];
  };


  struct MsgREF : public Message{
    MsgREF() : Message(Code::REF) {}
    MsgREF(uint8_t num) : Message(Code::REF, num) {}

    uint8_t getCount();
    int16_t getDeltaEnc(uint8_t index);

    bool setCount(uint8_t count);
    bool setDeltaEnc(uint8_t index, int16_t value);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);

  private:
    int16_t deltas[8];
  };


  struct MsgROBOT : public Message{
    MsgROBOT() : Message(Code::ROBOT) {}
    MsgROBOT(uint8_t num) : Message(Code::ROBOT, num) {}

    uint8_t getCount();
    uint32_t getTimeSampling();

    bool setCount(uint8_t count);
    bool setTimeSampling(uint32_t value);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);

  private:
    uint32_t timesampling_us;
  };


  struct MsgMOTOR : public Message{
    MsgMOTOR() : Message(Code::MOTOR) {}
    MsgMOTOR(uint8_t num) : Message(Code::MOTOR, num) {}
    
    uint8_t getIndex();
    bool getChangeEncoder();
    bool getInvertSpinDir();
    bool getChangeSpinDir();
    int8_t getSpinDirection();
    bool getInvertEncDir();
    bool getChangeEncDir();
    int8_t getEncDirection();
    int32_t getEncoderValue();

    bool setIndex(uint8_t index);
    bool setChangeEncoder(bool value);
    bool setInvertSpinDir(bool value);
    bool setChangeSpinDir(bool value);
    bool setSpinDirection(int8_t dir);
    bool setInvertEncDir(bool value);
    bool setChangeEncDir(bool value);
    bool setEncDirection(int8_t dir);
    bool setEncoderValue(int32_t value);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);

  private:
    uint8_t flags;
    int32_t encoder;
  };


  struct MsgPID : public Message{
    MsgPID() : Message(Code::PID) {}
    MsgPID(uint8_t num) : Message(Code::PID, num) {}
    
    uint8_t getIndex();
    float getPidDiv();
    float getPidKp();
    float getPidKi();
    float getPidKd();
    float getPidSat();
    float getPidPole();

    bool setIndex(uint8_t index);
    bool setPidDiv(float value);
    bool setPidKp(float value);
    bool setPidKi(float value);
    bool setPidKd(float value);
    bool setPidSat(float value);
    bool setPidPole(float value);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);

  private:
    float div;
    float kp;
    float ki;
    float kd;
    float sat;
    float pole;
  };


  struct MsgACKC : public Message{
    MsgACKC() : Message(Code::ACKC) {}
    MsgACKC(uint8_t num) : Message(Code::ACKC, num) {}

    uint8_t getCount();
    bool getEndStop(uint8_t index);
    int16_t getDeltaEnc(uint8_t index);
    
    bool setCount(uint8_t count);
    bool setEndStop(uint8_t index, bool value);
    bool setDeltaEnc(uint8_t index, int16_t value);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);

  private:
    uint8_t endstops;
    int16_t deltas[8];
  };


  struct MsgACKS : public Message{
    MsgACKS() : Message(Code::ACKS) {}
    MsgACKS(uint8_t num) : Message(Code::ACKS, num) {}

    uint8_t getCount();
    uint8_t getIndex();
    
    bool setCount(uint8_t count);
    bool setIndex(uint8_t index);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);
  };


  struct MsgERROR : public Message{
    MsgERROR() : Message(Code::ERROR) {}
    MsgERROR(uint8_t num) : Message(Code::ERROR, num) {}

    uint8_t getCount();
    uint8_t getIndex();
    
    bool setCount(uint8_t count);
    bool setIndex(uint8_t index);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);
  };


  static bool peek(Header *hdr, Timer *timeout_us = NULL);
  static bool rcv(Message *msg, Timer *timeout_us = NULL);
  static bool snd(Message *msg);
};


class RobotComm {
public:
  RobotComm(Robot &robot, uint8_t channel);
  ~RobotComm();

  void cycle(uint32_t time_us);
  void cycle();

private:
  Robot &robot;           // Robot controlled by this serial communication
  uint8_t channel;        // Serial channel used for communication

  Timer timer;            // Internal timer for serial communication during control operations
  Timer timeout;          // Internal timer for serial communication receiving timeouts
  long *encoders_rcv;     // Cumulative encoders values received
  long *encoders_snd;     // Cumulative encoders values sent
};

#endif  // COMMUNICATION_H