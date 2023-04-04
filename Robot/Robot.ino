// CODE for 1-8 DoF Robot control board

// ============================================================
// Includes
// ============================================================

#include "utils.h"
#include "control.h"


// ============================================================
// PINS DEFINITION
// ============================================================

// DC Motors PINs
#define MOTORS_EN 38    // Motors enabler

#define MOTOR_1_INA 22  // Motor 1 spin direction
#define MOTOR_1_INB 23  // Motor 1 spin direction
#define MOTOR_1_PWM 5   // Motor 1 spin pwm
#define MOTOR_1_CHA A8  // Motor 1 encoder A channel
#define MOTOR_1_CHB A14 // Motor 1 encoder B channel
#define MOTOR_1_END 37  // Motor 1 endstop switch

#define MOTOR_2_INA 24  // Motor 2 spin direction
#define MOTOR_2_INB 25  // Motor 2 spin direction
#define MOTOR_2_PWM 2   // Motor 2 spin pwm
#define MOTOR_2_CHA A9  // Motor 2 encoder A channel
#define MOTOR_2_CHB A15 // Motor 2 encoder B channel
#define MOTOR_2_END 36  // Motor 2 endstop switch

#define MOTOR_3_INA 26  // Motor 3 spin direction
#define MOTOR_3_INB 27  // Motor 3 spin direction
#define MOTOR_3_PWM 3   // Motor 3 spin pwm
#define MOTOR_3_CHA A10 // Motor 3 encoder A channel
#define MOTOR_3_CHB 10  // Motor 3 encoder B channel
#define MOTOR_3_END 35  // Motor 3 endstop switch

#define MOTOR_4_INA 28  // Motor 4 spin direction
#define MOTOR_4_INB 29  // Motor 4 spin direction
#define MOTOR_4_PWM 6   // Motor 4 spin pwm
#define MOTOR_4_CHA A11 // Motor 4 encoder A channel
#define MOTOR_4_CHB 11  // Motor 4 encoder B channel
#define MOTOR_4_END 34  // Motor 4 endstop switch

#define MOTOR_5_INA 49  // Motor 5 spin direction
#define MOTOR_5_INB 48  // Motor 5 spin direction
#define MOTOR_5_PWM 7   // Motor 5 spin pwm
#define MOTOR_5_CHA A12 // Motor 5 encoder A channel
#define MOTOR_5_CHB 12  // Motor 5 encoder B channel
#define MOTOR_5_END 33  // Motor 5 endstop switch

#define MOTOR_6_INA 47  // Motor 6 spin direction
#define MOTOR_6_INB 46  // Motor 6 spin direction
#define MOTOR_6_PWM 8   // Motor 6 spin pwm
#define MOTOR_6_CHA A13 // Motor 6 encoder A channel
#define MOTOR_6_CHB 13  // Motor 6 encoder B channel
#define MOTOR_6_END 32  // Motor 6 endstop switch

// Other pins
#define PIN_TOGGLE 52

// Control parameters
#define BAUDRATE 500000   // Serial baudrate
#define TS 10             // Control time sampling

#define ENC_1_DIV 1000.0  // Motor 1 divider for encorder error in PID mode
#define ENC_2_DIV 1000.0  // Motor 2 divider for encorder error in PID mode
#define ENC_3_DIV 1000.0  // Motor 3 divider for encorder error in PID mode
#define ENC_4_DIV 1000.0  // Motor 4 divider for encorder error in PID mode
#define ENC_5_DIV 1000.0  // Motor 5 divider for encorder error in PID mode
#define ENC_6_DIV 1000.0  // Motor 6 divider for encorder error in PID mode

#define PID_1_KP 1.0      // Motor 1 PID proportional coefficient
#define PID_1_KI 0.1      // Motor 1 PID integral coefficient
#define PID_1_KD 0.0      // Motor 1 PID derivative coefficient
#define PID_1_POLE 10.0   // Motor 1 PID dirty derivative pole
#define PID_1_SAT 100.0   // Motor 1 PID integral saturation

#define PID_2_KP 1.0      // Motor 2 PID proportional coefficient
#define PID_2_KI 0.1      // Motor 2 PID integral coefficient
#define PID_2_KD 0.0      // Motor 2 PID derivative coefficient
#define PID_2_POLE 10.0   // Motor 2 PID dirty derivative pole
#define PID_2_SAT 100.0   // Motor 2 PID integral saturation

#define PID_3_KP 1.0      // Motor 3 PID proportional coefficient
#define PID_3_KI 0.1      // Motor 3 PID integral coefficient
#define PID_3_KD 0.0      // Motor 3 PID derivative coefficient
#define PID_3_POLE 10.0   // Motor 3 PID dirty derivative pole
#define PID_3_SAT 100.0   // Motor 3 PID integral saturation

#define PID_4_KP 1.0      // Motor 4 PID proportional coefficient
#define PID_4_KI 0.1      // Motor 4 PID integral coefficient
#define PID_4_KD 0.0      // Motor 4 PID derivative coefficient
#define PID_4_POLE 10.0   // Motor 4 PID dirty derivative pole
#define PID_4_SAT 100.0   // Motor 4 PID integral saturation

#define PID_5_KP 1.0      // Motor 5 PID proportional coefficient
#define PID_5_KI 0.1      // Motor 5 PID integral coefficient
#define PID_5_KD 0.0      // Motor 5 PID derivative coefficient
#define PID_5_POLE 10.0   // Motor 5 PID dirty derivative pole
#define PID_5_SAT 100.0   // Motor 5 PID integral saturation

#define PID_6_KP 1.0      // Motor 6 PID proportional coefficient
#define PID_6_KI 0.1      // Motor 6 PID integral coefficient
#define PID_6_KD 0.0      // Motor 6 PID derivative coefficient
#define PID_6_POLE 10.0   // Motor 6 PID dirty derivative pole
#define PID_6_SAT 100.0   // Motor 6 PID integral saturation


// ============================================================
// COMPONENTS DEFINITION
// ============================================================

// DC Motor with Encoder
class Motor{
public:
  Motor(int INA, int INB, int PWM, int CHA, int CHB, int END){
    this->pin_INA = INA;
    this->pin_INB = INB;
    this->pin_PWM = PWM;
    this->pin_CHA = CHA;
    this->pin_CHB = CHB;
    this->pin_END = END;
    setPinMode();
    readEncoders();
  }

  ~Motor() {}

  // Motor operating modes
  enum OperatingMode {
    BRAKE_GND,
    SPIN_CCW,
    SPIN_CW,
    BRAKE_VCC
  };

  // Invert encoder ticks counting direction
  void invertEncoder(bool invert){
    this->encoder_invert = invert;
  }

  // Return encoder ticks
  long getEncoder(){
    return this->encoder;
  }

  // Set/reset encoder ticks
  void setEncoder(long value){
    this->encoder = value;
  }

  // Update encoder ticks
  void updateEncoder(){
    bool old_A = enc_A;
    readEncoders();
    if(old_A != enc_A){
      if(enc_B == enc_A) {
        encoder_invert ? encoder-- : encoder++;
      } else {
        encoder_invert ? encoder++ : encoder--;
      }
    }
  }

  // Invert physical spin direction of the motor
  void invertMotor(bool invert){
    this->motor_invert = invert;
  }

  // Assign pwm with sign for spin direction
  void driveMotor(short spwm){
    OperatingMode mode;

    if(spwm > 0) {
      mode = motor_invert ? SPIN_CCW : SPIN_CW;
    } else if (spwm < 0) {
      mode = motor_invert ? SPIN_CW : SPIN_CCW;
    } else {
      mode = BRAKE_GND;
    }
    
    opMotor(mode);
    pwmMotor(abs(spwm));
  }

  // Check if motor is at endstop
  bool isInEndStop(){
    return armState();
  }


private:
  int pin_INA;  // Input A pin
  int pin_INB;  // Input B pin
  int pin_PWM;  // Input pwm pin
  int pin_CHA;  // Output encoder channel A pin
  int pin_CHB;  // Output encoder channel B pin
  int pin_END;  // Output endstop switch pin

  bool enc_A;   // Encoder channel A state
  bool enc_B;   // Encoder channel B state

  bool encoder_invert = false;  // If invert encoder counting direction
  long encoder = 0;             // Encoder ticks count

  bool motor_invert = false;    // If invert motor physical spin direction

  // Set pin mode at initialization
  void setPinMode(){
    pinMode(pin_INA, OUTPUT);
    pinMode(pin_INB, OUTPUT);
    pinMode(pin_PWM, OUTPUT);
    pinMode(pin_CHA, INPUT_PULLUP);
    pinMode(pin_CHB, INPUT_PULLUP);
    pinMode(pin_END, INPUT);
  }

  // Read encoder pins
  void readEncoders(){
    enc_A = digitalRead(pin_CHA);
    enc_B = digitalRead(pin_CHB);
  }

  // Set motor operating mode (spin direction and hold)
  void opMotor(OperatingMode mode){
    switch(mode){
      case BRAKE_GND:
        digitalWrite(pin_INA, LOW);
        digitalWrite(pin_INB, LOW);
        break;
      case SPIN_CCW:
        digitalWrite(pin_INA, LOW);
        digitalWrite(pin_INB, HIGH);
        break;
      case SPIN_CW:
        digitalWrite(pin_INA, HIGH);
        digitalWrite(pin_INB, LOW);
        break;
      case BRAKE_VCC:
        digitalWrite(pin_INA, HIGH);
        digitalWrite(pin_INB, HIGH);
        break;
    }
  }

  // Set motor pwm value
  void pwmMotor(unsigned int pwm){
    analogWrite(pin_PWM, pwm);
  }

  // Read endstop pin state
  bool armState(){
    return (digitalRead(pin_END) == HIGH) ? true : false;
  }
};



// Serial Communication Protocol (static)
class Communication {
public:
  Communication() = delete;
  ~Communication() = delete;
  
  // Store received control message data
  struct RCVctrl {
    RCVctrl() = delete;

    RCVctrl(unsigned char n){
      this->values = malloc(n * sizeof(short));
    }

    ~RCVctrl(){
      free(this->values);
    }

    unsigned char num;
    unsigned char command;
    short *values;
  };

  // Store sending control response data
  struct SNDctrl {
    SNDctrl() = delete;

    SNDctrl(unsigned char n){
      this->switches = malloc(n * sizeof(bool));
      this->values = malloc(n * sizeof(short));
    }

    ~SNDctrl(){
      free(this->switches);
      free(this->values);
    }

    unsigned char num;
    unsigned char status;
    bool *switches;
    short *values;
  };

  // Store received setup message data
  struct RCVsetup {
    RCVsetup(){
      this->values = malloc(6 * sizeof(float));
    }

    ~RCVsetup(){
      free(this->values);
    }

    unsigned char num;
    unsigned char command;
    float *values;
  };

  // Store sending setup response data
  struct SNDsetup {
    SNDsetup(){
    }

    ~SNDsetup(){
    }

    unsigned char status;
  };
  
  // Describe next serial bytes kind of message
  enum class Next {
    Error = -1,
    None  = 0,
    Ctrl  = 1,
    Setup = 2
  };

  // Check next byte without removing from serial buffer to return next message type
  static Next peek(){
    int res = -1;

    if(Serial.available() > 0){
      res = Serial.peek();
    }

    if(res == -1) {
      return Next::None;
    } else {
      unsigned char byte = res;
      byte = byte & 0b00011111;
      if(byte == 0 || byte == 1 || byte == 2){
        return Next::Ctrl;
      } else if(byte == 3) {
        return Next::Setup;
      } else {
        return Next::Error;
      }
    }
  }

  // Fill rcv_ctrl with incoming control message bytes
  static void rcv(RCVctrl *rcv_ctrl){
    unsigned char byte;
    while(Serial.available() < 1);
    byte = Serial.read();

    rcv_ctrl->num = (byte >> 5) + 1;
    rcv_ctrl->command = byte & 0b00011111;

    while(Serial.available() < 1);
    byte = Serial.read();
  
    for (int i = 0; i < rcv_ctrl->num; i++) {
      while(Serial.available() < 1);
      rcv_ctrl->values[i] = (1 - 2 *((byte >> i) & 0b00000001)) * ((short) Serial.read());
    }
  }

  // Send snd_ctrl control response data through serial
  static void snd(SNDctrl *snd_ctrl){
    buffer[0] = (((unsigned char) (snd_ctrl->num - 1)) << 5) | (snd_ctrl->status & 0b00011111);
    buffer[1] = 0b00000000;
    buffer[2] = 0b00000000;

    for(int i = 0; i < snd_ctrl->num; i++) {
      buffer[1] = buffer[1] | (snd_ctrl->switches[i] ? (0b00000001 << i) : 0b00000000);
      buffer[2] = buffer[2] | ((snd_ctrl->values[i] < 0) ? (0b00000001 << i) : 0b00000000);
      buffer[3+i] = (unsigned char) min(abs(snd_ctrl->values[i]), 255);
    }

    for (int i = 0; i < 3+snd_ctrl->num; i++) {
      Serial.write(buffer[i]);
    }
  }

  // Fill rcv_setup with incoming setup message bytes
  static void rcv(RCVsetup *rcv_setup){
    unsigned char byte;
    while(Serial.available() < 1);
    byte = Serial.read();

    rcv_setup->num = (byte >> 5);
    rcv_setup->command = byte & 0b00011111;

    for (int i = 0; i < 6; i++){
      rcv_setup->values[i] = 0b0;
      unsigned char bytes[4] = {0b0, 0b0, 0b0, 0b0};
      for (int j = 0; j < 4; j++){
        while(Serial.available() < 1);
        bytes[j] = Serial.read();
      }
      rcv_setup->values[i] = *((float *) bytes);
    }
  }

  // Send snd_setup setup response data through serial
  static void snd(SNDsetup *snd_setup){
    unsigned char byte = snd_setup->status & 0b00011111;
    Serial.write(byte);
  }


private:
  inline static unsigned char buffer[32];   // Internal buffer for serial read/write operations
};



// 1-8 DoF Robot with DC Motors
class Robot {
public:
  Robot(unsigned char size, int EN, Motor **motors, float *encs_div){
    this->motors = (Motor**) malloc(size * sizeof(Motor*));
    this->pids = (PID*) malloc(size * sizeof(PID));
    this->switches = malloc(size * sizeof(bool));
    this->motors_pwm = malloc(size * sizeof(short));
    this->encoders_rcv = malloc(size * sizeof(long));
    this->encoders_snd = malloc(size * sizeof(long));
    this->error_div = malloc(size * sizeof(float));
    
    this->size = size;
    this->pin_EN = EN;

    this->status = Status::Idle;

    for(int i = 0; i < size; i++){
      this->switches[i] = false;
      this->motors_pwm[i] = 0;
      this->encoders_snd[i] = 0;
      this->encoders_rcv[i] = 0;
      this->error_div[i] = 0.0;
    }

    this->snd_ctrl = new Communication::SNDctrl(size);
    this->rcv_ctrl = new Communication::RCVctrl(size);
    this->snd_setup = new Communication::SNDsetup();
    this->rcv_setup = new Communication::RCVsetup();
    
    if(motors != NULL){
      for(int i = 0; i < size; i++){
        if(encs_div != NULL){
          setMotor(i, motors[i], encs_div[i]);
        } else {
          setMotor(i, motors[i]);
        }
      }
    }

    setPinMode();
    update();
  }

  Robot(unsigned char size, int EN, Motor **motors) : Robot(size, EN, motors, NULL) {}

  Robot(unsigned char size, int EN) : Robot(size, EN, NULL, NULL) {}


  ~Robot() {
    free(this->motors);
    free(this->pids);
    free(this->switches);
    free(this->motors_pwm);
    free(this->encoders_rcv);
    free(this->encoders_snd);
    free(this->error_div);
    delete this->snd_ctrl;
    delete this->rcv_ctrl;
    delete this->snd_setup;
    delete this->rcv_setup;
  }

  // Robot possible commands
  enum class Command : unsigned char {
    Idle = 0,
    DAQ = 1,
    PID = 2,
    Setup = 3
  };

  // Robot possible statuses
  enum class Status : unsigned char {
    Idle = 0,
    DAQ = 1,
    PID = 2,
    Setup = 3
  };

  // Return number of motors
  int getSize(){
    return this->size;
  }

  // Return robot status
  Status getStatus(){
    return this->status;
  }

  // Return pointer to motor of specified index
  Motor * getMotor(int index){
    return this->motors[index];
  }

  // Set robot's motor of specified index
  void setMotor(int index, Motor * motor){
    this->motors[index] = motor;
  }

  // Set robot's motor of specified index and relative encoder divider
  void setMotor(int index, Motor * motor, float enc_div){
    this->motors[index] = motor;
    this->error_div[index] = enc_div;
  }

  // Set motor's encoder divider for specified index
  void setEncoderDivider(int index, float enc_div){
    this->error_div[index] = enc_div;
  }

  // Return pointer to motor's PID specified by index
  PID * getPID(int index){
    return &(this->pids[index]);
  }

  // Initialize all PIDs with following parameters
  void initPIDs(float ts, float pole, float sat, bool bumpless){
    for(int i = 0; i < size; i++){
      getPID(i)->init(ts, pole, sat, bumpless);
    }
  }

  // Initialize all PIDs with following coefficients
  void setupPIDs(float kp, float ki, float kd){
    for(int i = 0; i < size; i++){
      getPID(i)->setup(kp, ki, kd);
    }
  }

  // Reset all PIDs state
  void resetPIDs(){
    for(int i = 0; i < size; i++){
      getPID(i)->reset();
    }
  }

  // Update all motors' encoders
  void updateEncoders(){
    for(int i = 0; i < size; i++){
      getMotor(i)->updateEncoder();
    }
  }

  // Set motors' PWMs values
  void setPWMs(short *pwms){
    for(int i = 0; i < size; i++){
      setPWM(i, pwms[i]);
    }
  }

  // Set PWM value for motor specified by index 
  void setPWM(int index, short pwm){
    motors_pwm[index] = (short) constrain(pwm, -255, 255);
  }

  // Reset all motors' PWMs values to zero
  void resetPWMs(){
    for(int i = 0; i < size; i++){
      motors_pwm[i] = 0;
    }
  }

  // Enable all motors
  void enableMotors(){
    setStatus(Status::Idle, true);
    setEnabler(true);
  }

  // Disable all motors
  void disableMotors(){
    setStatus(Status::Idle, true);
    setEnabler(false);
  }

  // Check next message type
  Communication::Next peek(){
    return Communication::peek();
  }

  // Receive and process a control message from serial communication
  void rcvCtrl(){
    Communication::rcv(rcv_ctrl);
    
    if(rcv_ctrl->command > 2 || rcv_ctrl->num != size) {
      setStatus(Status::Idle, true);
      return;
    }

    switch(rcv_ctrl->command){
      case (unsigned char) Command::Idle:
        for(int i = 0; i < size; i++) {
          motors_pwm[i] = 0;
          encoders_rcv[i] = 0;
        }
        break;
      case (unsigned char) Command::DAQ:
        for(int i = 0; i < size; i++) {
          motors_pwm[i] = rcv_ctrl->values[i];
          encoders_rcv[i] = 0;
        }
        break;
      case (unsigned char) Command::PID:
        for(int i = 0; i < size; i++) {
          motors_pwm[i] = 0;
          encoders_rcv[i] += rcv_ctrl->values[i];
        }
        break;
    }

    setStatus((Status) rcv_ctrl->command);
  }

  // Make and send a control response to serial communication
  void sndCtrl(){
    snd_ctrl->num = size;
    snd_ctrl->status = (unsigned char) status;

    for(int i = 0; i < size; i++) {
      snd_ctrl->switches[i] = switches[i];
      snd_ctrl->values[i] = min(getMotor(i)->getEncoder() - encoders_snd[i], 255);
      encoders_snd[i] += snd_ctrl->values[i];
    }
    
    Communication::snd(snd_ctrl);
  }

  // Receive and process a setup message from serial communication
  void rcvSetup(){
    Communication::rcv(rcv_setup);
    
    setEncoderDivider(rcv_setup->num, rcv_setup->values[0]);
    getPID(rcv_setup->num)->reset();
    getPID(rcv_setup->num)->init(TS/1000.0, rcv_setup->values[4], rcv_setup->values[5], true);
    getPID(rcv_setup->num)->setup(rcv_setup->values[1], rcv_setup->values[2], rcv_setup->values[3]);

    setStatus(Status::Idle, true);

    return rcv_setup->num;
  }

  // Make and send a setup response to serial communication
  void sndSetup(){
    snd_setup->status = (unsigned char) Status::Setup;
    Communication::snd(snd_setup);
  }

  // Update motors' PWMs values according to current robot state
  void update(){
    switch(status){
      case Status::Idle:
        resetPWMs();
        break;

      case Status::DAQ:
        break;

      case Status::PID:
        feedback();
        break;

      default:
        resetPWMs();
        resetPIDs();
        status = Status::Idle;
        break;
    }

    for(int i = 0; i < size; i++){
      switches[i] = getMotor(i)->isInEndStop();
    }
  }

  // Set current PWM values for motors
  void actuate(){
    for(int i = 0; i < size; i++){
      getMotor(i)->driveMotor(motors_pwm[i]);
    }
  }

  
private:
  unsigned char size; // Number of motors
  int pin_EN;         // Output pin for motors enabling/disabling
  Motor **motors;     // Pointers array to Motors
  PID *pids;          // Pointer to motors' PIDs

  Status status;      // Robot status
  bool *switches;     // Motors' endstops switches values
  short *motors_pwm;  // Motors' PWMs current values

  long *encoders_snd; // Cumulative encoders values sent
  long *encoders_rcv; // Cumulative encoders values received
  float *error_div;   // Encoders error dividers
  
  Communication::SNDctrl *snd_ctrl;   // Control message data
  Communication::RCVctrl *rcv_ctrl;   // Control response data
  Communication::SNDsetup *snd_setup; // Setup message data
  Communication::RCVsetup *rcv_setup; // Setup response data
  
  // Set pin mode at initialization
  void setPinMode(){
    pinMode(pin_EN, OUTPUT);
  }
  
  // Set motors enabler pin state
  void setEnabler(bool value){
    digitalWrite(pin_EN, value ? HIGH : LOW);
  }

  // Set robot internal status
  void setStatus(Status status, bool reset = false){
    if(this->status != status || reset){
      resetPWMs();
      resetPIDs();
      this->status = status;
    }
  }

  // Elaborate a feedback control step
  void feedback(){
    for(int i = 0; i < size; i++){
      float err = (float) (getMotor(i)->getEncoder() - encoders_rcv[i]) / ((error_div[i] == 0) ? 1.0 : error_div[i]);
      motors_pwm[i] = (short) constrain(getPID(i)->evolve(err), -255.0, 255.0);
    }
  }
};



// ============================================================
// PWM frequency
// ============================================================

// PWMs pin frequency values
enum class FrequencyPWM : unsigned char {
  FREQ_31372_55 = 0b00000001,
  FREQ_3921_16  = 0b00000010,
  FREQ_490_20 = 0b00000011,
  FREQ_122_55 = 0b00000100,
  FREQ_30_64  = 0b00000101
};

// Set PWMs pins frequency
void setFrequencyPWM(FrequencyPWM freq){
  TCCR3B = (TCCR3B & 0b11111000) | ((unsigned char) freq);   // PWM frequency for pin D2, D3 & D5
  TCCR4B = (TCCR4B & 0b11111000) | ((unsigned char) freq);   // PWM frequency for pin D6, D7 & D8
}



// ============================================================
// Components
// ============================================================

/*
Motor motor1 = Motor(MOTOR_1_INA, MOTOR_1_INB, MOTOR_1_PWM, MOTOR_1_CHA, MOTOR_1_CHB, MOTOR_1_END);
Motor motor2 = Motor(MOTOR_2_INA, MOTOR_2_INB, MOTOR_2_PWM, MOTOR_2_CHA, MOTOR_2_CHB, MOTOR_2_END);
Motor motor3 = Motor(MOTOR_3_INA, MOTOR_3_INB, MOTOR_3_PWM, MOTOR_3_CHA, MOTOR_3_CHB, MOTOR_3_END);
Motor motor4 = Motor(MOTOR_4_INA, MOTOR_4_INB, MOTOR_4_PWM, MOTOR_4_CHA, MOTOR_4_CHB, MOTOR_4_END);
Motor motor5 = Motor(MOTOR_5_INA, MOTOR_5_INB, MOTOR_5_PWM, MOTOR_5_CHA, MOTOR_5_CHB, MOTOR_5_END);
Motor motor6 = Motor(MOTOR_6_INA, MOTOR_6_INB, MOTOR_6_PWM, MOTOR_6_CHA, MOTOR_6_CHB, MOTOR_6_END);

Motor *motors[6] = {&motor1, &motor2, &motor3, &motor4, &motor5, &motor6};
Robot robot = Robot(6, MOTORS_EN, motors);
*/

Robot robot = Robot(6, MOTORS_EN);  // Robot
Timer timer = Timer(TS);            // Control timer
bool first_ctrl = true;             // If is first time receving control message


// ============================================================
// Arduino setup
// ============================================================

// Setup
void setup()
{
  pinMode(PIN_TOGGLE, OUTPUT);

  setFrequencyPWM(FrequencyPWM::FREQ_3921_16);

  Serial.begin(BAUDRATE);
  Serial.flush();

  robot.setMotor(0, new Motor(MOTOR_1_INA, MOTOR_1_INB, MOTOR_1_PWM, MOTOR_1_CHA, MOTOR_1_CHB, MOTOR_1_END));
  robot.setMotor(1, new Motor(MOTOR_2_INA, MOTOR_2_INB, MOTOR_2_PWM, MOTOR_2_CHA, MOTOR_2_CHB, MOTOR_2_END));
  robot.setMotor(2, new Motor(MOTOR_3_INA, MOTOR_3_INB, MOTOR_3_PWM, MOTOR_3_CHA, MOTOR_3_CHB, MOTOR_3_END));
  robot.setMotor(3, new Motor(MOTOR_4_INA, MOTOR_4_INB, MOTOR_4_PWM, MOTOR_4_CHA, MOTOR_4_CHB, MOTOR_4_END));
  robot.setMotor(4, new Motor(MOTOR_5_INA, MOTOR_5_INB, MOTOR_5_PWM, MOTOR_5_CHA, MOTOR_5_CHB, MOTOR_5_END));
  robot.setMotor(5, new Motor(MOTOR_6_INA, MOTOR_6_INB, MOTOR_6_PWM, MOTOR_6_CHA, MOTOR_6_CHB, MOTOR_6_END));

  robot.setEncoderDivider(0, ENC_1_DIV);
  robot.setEncoderDivider(0, ENC_2_DIV);
  robot.setEncoderDivider(0, ENC_3_DIV);
  robot.setEncoderDivider(0, ENC_4_DIV);
  robot.setEncoderDivider(0, ENC_5_DIV);
  robot.setEncoderDivider(0, ENC_6_DIV);
  
  robot.getPID(0)->init((float) TS/1000.0, PID_1_POLE, PID_1_SAT, true);
  robot.getPID(1)->init((float) TS/1000.0, PID_2_POLE, PID_2_SAT, true);
  robot.getPID(2)->init((float) TS/1000.0, PID_3_POLE, PID_3_SAT, true);
  robot.getPID(3)->init((float) TS/1000.0, PID_4_POLE, PID_4_SAT, true);
  robot.getPID(4)->init((float) TS/1000.0, PID_5_POLE, PID_5_SAT, true);
  robot.getPID(5)->init((float) TS/1000.0, PID_6_POLE, PID_6_SAT, true);

  robot.getPID(0)->setup(PID_1_KP, PID_1_KI, PID_1_KD);
  robot.getPID(1)->setup(PID_2_KP, PID_2_KI, PID_2_KD);
  robot.getPID(2)->setup(PID_3_KP, PID_3_KI, PID_3_KD);
  robot.getPID(3)->setup(PID_4_KP, PID_4_KI, PID_4_KD);
  robot.getPID(4)->setup(PID_5_KP, PID_5_KI, PID_5_KD);
  robot.getPID(5)->setup(PID_6_KP, PID_6_KI, PID_6_KD);

  robot.enableMotors();

  digitalWrite(PIN_TOGGLE, LOW);
  
  timer.reset(millis());
}



// ============================================================
// Arduino Loop
// ============================================================

// Loop
void loop()
{
  unsigned long time = millis();
  Communication::Next next = robot.peek();

  if(next == Communication::Next::Setup){
    robot.rcvSetup();
    robot.sndSetup();
    timer.reset(time);
    first_ctrl = true;
  } else if (next == Communication::Next::Error) {
    Serial.flush();
    timer.reset(time);
    first_ctrl = true;
  } else {
    if(first_ctrl && next == Communication::Next::Ctrl){
      digitalWrite(PIN_TOGGLE, HIGH);
      robot.rcvCtrl();
      robot.update();
      robot.actuate();
      digitalWrite(PIN_TOGGLE, LOW);
      timer.reset(time);
      if(robot.getStatus() != Robot::Status::Idle) {
        first_ctrl = false;
      } else {
        robot.sndCtrl();
      }
    } else if (!first_ctrl) {
      if (timer.check(time)){
        digitalWrite(PIN_TOGGLE, HIGH);
        robot.sndCtrl();
        robot.rcvCtrl();
        robot.update();
        robot.actuate();
        digitalWrite(PIN_TOGGLE, LOW);
        if(robot.getStatus() == Robot::Status::Idle){
          robot.sndCtrl();
          first_ctrl = true;
        }
      }
    }
  } 
}
