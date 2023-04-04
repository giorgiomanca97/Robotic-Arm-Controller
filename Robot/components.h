#ifndef COMPONENTS_H
#define COMPONENTS_H

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define MEGA
#endif

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
#define UNO
#endif

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
#define LEONARDO
#endif

//#define PIN_EXTRA_FEATURES

#include <Arduino.h>
#include <math.h>
#include "utils.h"
#include "control.h"
#include "communication.h"


#if defined(UNO) || defined(MEGA)
class PWMfreq {
public:
#if defined(UNO)
  enum class UnoTimer0 : unsigned char{   // D5 & D6
    FREQ_62500_00 = 0b00000001, // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
    FREQ_7812_50  = 0b00000010, // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
    FREQ_976_56   = 0b00000011, // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz (DEFAULT)
    FREQ_244_14   = 0b00000100, // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
    FREQ_61_04    = 0b00000101  // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz
  };

  enum class UnoTimer1 : unsigned char{   // D9 & D10 
    FREQ_31372_55 = 0b00000001, // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_490_20   = 0b00000011, // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_122_55   = 0b00000100, // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000101  // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
  };

  enum class UnoTimer2 : unsigned char{   // D3 & D11
    FREQ_31372_55 = 0b00000001, // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_980_39   = 0b00000011, // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
    FREQ_490_20   = 0b00000100, // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_245_10   = 0b00000101, // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
    FREQ_122_55   = 0b00000110, // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000111  // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
  };
#endif

#if defined(MEGA)
  enum class MegaTimer0 : unsigned char{   // D4 & D13
    FREQ_62500_00 = 0b00000001, // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
    FREQ_7812_50  = 0b00000010, // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
    FREQ_976_56   = 0b00000011, // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz (DEFAULT)
    FREQ_244_14   = 0b00000100, // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
    FREQ_61_04    = 0b00000101  // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz
  };

  enum class MegaTimer1 : unsigned char{   // D11 & D12 
    FREQ_31372_55 = 0b00000001, // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_490_20   = 0b00000011, // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_122_55   = 0b00000100, // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000101  // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
  };

  enum class MegaTimer2 : unsigned char{   // D9 & D10
    FREQ_31372_55 = 0b00000001, // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_980_39   = 0b00000011, // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
    FREQ_490_20   = 0b00000100, // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_245_10   = 0b00000101, // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
    FREQ_122_55   = 0b00000110, // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000111  // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
  };

  enum class MegaTimer3 : unsigned char{   // D2, D3 & D5 
    FREQ_31372_55 = 0b00000001, // set timer 3 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 3 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_490_20   = 0b00000011, // set timer 3 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_122_55   = 0b00000100, // set timer 3 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000101  // set timer 3 divisor to  1024 for PWM frequency of    30.64 Hz
  };  

  enum class MegaTimer4 : unsigned char{   // D6, D7 & D8 
    FREQ_31372_55 = 0b00000001, // set timer 4 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 4 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_490_20   = 0b00000011, // set timer 4 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_122_55   = 0b00000100, // set timer 4 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000101  // set timer 4 divisor to  1024 for PWM frequency of    30.64 Hz
  };

  enum class MegaTimer5 : unsigned char{   // D44, D45 & D46 
    FREQ_31372_55 = 0b00000001, // set timer 5 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 5 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_490_20   = 0b00000011, // set timer 5 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_122_55   = 0b00000100, // set timer 5 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000101  // set timer 5 divisor to  1024 for PWM frequency of    30.64 Hz
  };
#endif

  //NOTE: Changing this timer 0 affects millis() and delay!

#if defined(UNO)
  static void set(UnoTimer0 freq);
  static void set(UnoTimer1 freq);
  static void set(UnoTimer2 freq);
#endif

#if defined(MEGA)
  static void set(MegaTimer0 freq);
  static void set(MegaTimer1 freq);
  static void set(MegaTimer2 freq);
  static void set(MegaTimer3 freq);
  static void set(MegaTimer4 freq);
  static void set(MegaTimer5 freq);
#endif
};
#endif


class PinControl {
public:
  PinControl(uint8_t pin);
  PinControl(uint8_t pin, float v1, float v2);
  
  void setLimits(float v1, float v2);

  void set(bool state);
  void pwm(uint8_t pwm);
  void control(float value);
  #if defined(PIN_EXTRA_FEATURES)
  void feedback(float error);

  void setPID(PID *pid);
  PID* getPID();
  #endif

private:
  uint8_t pin;
  float v1;
  float v2;
  #if defined(PIN_EXTRA_FEATURES)
  PID *pid;
  #endif
};


class PinMeasure {
public:
  PinMeasure(uint8_t pin, bool pullup = false);
  PinMeasure(uint8_t pin, float v1, float v2, bool pullup = false);
  
  void setLimits(float v1, float v2);

  bool state();
  uint16_t value();
  float measure();
  #if defined(PIN_EXTRA_FEATURES)
  float filter();

  void setFilter(Filter *filter);
  Filter* getFilter();
  #endif

private:
  uint8_t pin;
  float v1;
  float v2;
  #if defined(PIN_EXTRA_FEATURES)
  Filter* fil;
  #endif
};


// DC Motor with Encoder
class Motor{
public:
  Motor(PinControl &INA, PinControl &INB, PinControl &PWM, PinMeasure &CHA, PinMeasure &CHB, PinMeasure &END);
  ~Motor();

  // Motor operating modes
  enum class OperatingMode {
    BRAKE_GND,
    SPIN_CCW,
    SPIN_CW,
    BRAKE_VCC
  };

  void invertEncoder(bool invert);  // Invert encoder ticks counting direction
  long getEncoder();                // Return encoder ticks
  void setEncoder(long value);      // Set/reset encoder ticks
  void readEncoder();               // Read encoder pins
  void updateEncoder();             // Update encoder ticks

  void invertMotor(bool invert);    // Invert physical spin direction of the motor
  void driveMotor(short spwm);      // Assign pwm with sign for spin direction

  bool isInEndStop();               // Check if motor is at endstop

private:
  PinControl &pin_INA;          // A pin
  PinControl &pin_INB;          // B pin
  PinControl &pin_PWM;          // pwm pin
  PinMeasure &pin_CHA;          // encoder channel A
  PinMeasure &pin_CHB;          // encoder channel B
  PinMeasure &pin_END;          // endstop switch

  bool enc_A = false;           // Encoder channel A state
  bool enc_B = false;           // Encoder channel B state

  bool encoder_invert = false;  // If invert encoder counting direction
  long encoder = 0;             // Encoder ticks count

  bool motor_invert = false;    // If invert motor physical spin direction
};


// 1-8 DoF Robot with DC Motors
class Robot {
public:
  Robot(PinControl &enable, PinControl &toggle, unsigned long ts_ms, uint8_t size, Motor **motors, float *encs_div);
  Robot(PinControl &enable, PinControl &toggle, unsigned long ts_ms, uint8_t size, Motor **motors);
  Robot(PinControl &enable, PinControl &toggle, unsigned long ts_ms, uint8_t size);
  ~Robot();

  // Robot possible commands
  enum class Command : uint8_t {
    Idle = 0,
    DAQ = 1,
    PID = 2,
    Setup = 3
  };

  // Robot possible statuses
  enum class Status : uint8_t {
    Idle = 0,
    DAQ = 1,
    PID = 2,
    Setup = 3
  };

  int getSize();                                      // Return number of motors
  Status getStatus();                                 // Return robot status
  void setStatus(Status status, bool reset = false);  // Set robot internal status

  Motor * getMotor(uint8_t index);                              // Return pointer to motor of specified index
  void setMotor(uint8_t index, Motor * motor);                  // Set robot's motor of specified index
  void setMotor(uint8_t index, Motor * motor, float enc_div);   // Set robot's motor of specified index and relative encoder divider
  void setEncoderDivider(uint8_t index, float enc_div);         // Set motor's encoder divider for specified index

  PID * getPID(uint8_t index);                                    // Return pointer to motor's PID specified by index
  void initPIDs(float ts, float pole, float sat, bool bumpless);  // Initialize all PIDs with following parameters
  void setupPIDs(float kp, float ki, float kd);                   // Initialize all PIDs with following coefficients
  void resetPIDs();                                               // Reset all PIDs state
  
  void updateEncoders();                        // Update all motors' encoders
  void setEncoders(long *values);               // Set encoders' values
  void setEncoder(uint8_t index, long value);   // Set encoder value for motor specified by index 
  void resetEncoders();                         // Reset all motors' encoders
  
  void setPWMs(short *pwms);                    // Set motors' PWMs values
  void setPWM(uint8_t index, short pwm);        // Set PWM value for motor specified by index 
  void resetPWMs();                             // Reset all motors' PWMs values to zero

  void enableMotors();                          // Enable all motors
  void disableMotors();                         // Disable all motors
  
  Communication::Next peek();         // Check next message type
  void rcvCtrl();                     // Receive and process a control message from serial communication
  void sndCtrl();                     // Make and send a control response to serial communication
  void rcvSetup();                    // Receive and process a setup message from serial communication
  void sndSetup();                    // Make and send a setup response to serial communication

  void update();                      // Update robot internal state according to status and received data
  void actuate();                     // Apply computed PWMs
  void cycle(unsigned long time_ms);  // Looping function
  
  
private:
  PinControl &pin_enable; // Pin for motors enabling/disabling
  PinControl &pin_toggle; // Pin for motors enabling/disabling

  unsigned long ts;       // Time sampling in milliseconds
  unsigned char size;     // Number of motors
  Motor **motors;         // Pointers array to Motors
  PID *pids;              // Pointer to motors' PIDs
  
  Status status;          // Robot status
  Timer timer;            // Robot cycle timer
  
  bool *switches;         // Motors' endstops switches values
  short *motors_pwm;      // Motors' PWMs current values

  long *encoders_snd;     // Cumulative encoders values sent
  long *encoders_rcv;     // Cumulative encoders values received
  float *error_div;       // Encoders error dividers
  
  Communication::SNDctrl *snd_ctrl;     // Control message data
  Communication::RCVctrl *rcv_ctrl;     // Control response data
  Communication::SNDsetup *snd_setup;   // Setup message data
  Communication::RCVsetup *rcv_setup;   // Setup response data
};


#endif  // COMPONENTS_H