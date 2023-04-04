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
    Code getCode();
    uint8_t getNum();

    bool setCode(Code code);
    bool setNum(uint8_t num);

    bool parse(uint8_t byte);
    uint8_t byte();

    uint8_t from(uint8_t *buffer);
    uint8_t fill(uint8_t *buffer);

  private:
    Code code;
    uint8_t num;
  };

  
  struct Message{
    Message(Code code);

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

    uint8_t getCount();
    
    bool setCount(uint8_t count);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);
  };


  struct MsgPWM : public Message{
    MsgPWM() : Message(Code::PWM) {}

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
    
    uint8_t getIndex();
    bool getChangeEncoder();
    bool getInvertSpinDir();
    bool getChangeSpinDir();
    int8_t getSpinDirection();
    bool getInvertEncDir();
    bool getChangeEncDir();
    int8_t getEncDirection();
    uint32_t getEncoderValue();

    bool setIndex(uint8_t index);
    bool setChangeEncoder(bool value);
    bool setInvertSpinDir(bool value);
    bool setChangeSpinDir(bool value);
    bool setSpinDirection(int8_t dir);
    bool setInvertEncDir(bool value);
    bool setChangeEncDir(bool value);
    bool setEncDirection(int8_t dir);
    bool setEncoderValue(uint32_t value);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);

  private:
    uint8_t flags;
    uint32_t encoder;
  };


  struct MsgPID : public Message{
    MsgPID() : Message(Code::PID) {}
    
    uint8_t getIndex();
    float getPidDiv();
    float getPidKp();
    float getPidKi();
    float getPidKd();
    float getPidPole();
    float getPidSat();

    bool setIndex(uint8_t index);
    bool setPidDiv(float value);
    bool setPidKp(float value);
    bool setPidKi(float value);
    bool setPidKd(float value);
    bool setPidPole(float value);
    bool setPidSat(float value);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);

  private:
    float div;
    float kp;
    float ki;
    float kd;
    float pole;
    float sat;
  };


  struct MsgACKC : public Message{
    MsgACKC() : Message(Code::ACKC) {}

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
  static void flush();
};


class RobotComm {
public:
  RobotComm(Robot &robot, PinControl &toggle, uint8_t channel);
  ~RobotComm();

  void cycle(uint32_t time_us);

private:
  Robot &robot;           // Robot controlled by this serial communication
  PinControl &toggle;     // Pin used to debug time sampling consistency
  uint8_t channel;        // Serial channel used for communication

  Timer timer;            // Internal timer for serial communication during control operations
  Timer timeout;          // Internal timer for serial communication receiving timeouts
  long *encoders_rcv;     // Cumulative encoders values received
  long *encoders_snd;     // Cumulative encoders values sent
};

#endif  // COMMUNICATION_H