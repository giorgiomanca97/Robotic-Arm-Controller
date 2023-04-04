#ifndef COMMUNICATION_H
#define COMMUNICATION_H


#include <Arduino.h>
#include <math.h>
#include "utils.h"


// Serial Communication Protocol (static)
class Communication {
public:
  Communication() = delete;
  ~Communication() = delete;
  
  // Store received control message data
  struct RCVctrl {
    RCVctrl() = delete;
    RCVctrl(unsigned char n);
    ~RCVctrl();

    unsigned char num;
    unsigned char command;
    short *values;
  };

  // Store sending control response data
  struct SNDctrl {
    SNDctrl() = delete;
    SNDctrl(unsigned char n);
    ~SNDctrl();

    unsigned char num;
    unsigned char status;
    bool *switches;
    short *values;
  };

  // Store received setup message data
  struct RCVsetup {
    RCVsetup();
    ~RCVsetup();

    unsigned char num;
    unsigned char command;
    float *values;
  };

  // Store sending setup response data
  struct SNDsetup {
    SNDsetup();
    ~SNDsetup();

    unsigned char status;
  };
  
  // Describe next serial bytes kind of message
  enum class Next : int {
    Error = -1,
    None  = 0,
    Ctrl  = 1,
    Setup = 2
  };

  static Next peek();                     // Check next byte without removing from serial buffer to return next message type

  static void rcv(RCVctrl *rcv_ctrl);     // Fill rcv_ctrl with incoming control message bytes
  static void snd(SNDctrl *snd_ctrl);     // Send snd_ctrl control response data through serial
  static void rcv(RCVsetup *rcv_setup);   // Fill rcv_setup with incoming setup message bytes
  static void snd(SNDsetup *snd_setup);   // Send snd_setup setup response data through serial

private:
  inline static unsigned char buffer[16];
};


#endif  // COMMUNICATION_H