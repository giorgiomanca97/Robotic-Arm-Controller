#include "components.h"


// ==================================================
// PWMfreq
// ==================================================

#if defined(UNO) || defined(MEGA)

#if defined(UNO)
static void PWMfreq::set(UnoTimer0 freq){
  TCCR0B = (TCCR0B & 0b11111000) | ((uint8_t) freq);
}
static void PWMfreq::set(UnoTimer1 freq){
  TCCR1B = (TCCR1B & 0b11111000) | ((uint8_t) freq);
}
static void PWMfreq::set(UnoTimer2 freq){
  TCCR2B = (TCCR2B & 0b11111000) | ((uint8_t) freq);
}
#endif

#if defined(MEGA)
static void PWMfreq::set(MegaTimer0 freq){
  TCCR0B = (TCCR0B & 0b11111000) | ((uint8_t) freq);
}
static void PWMfreq::set(MegaTimer1 freq){
  TCCR1B = (TCCR1B & 0b11111000) | ((uint8_t) freq);
}
static void PWMfreq::set(MegaTimer2 freq){
  TCCR2B = (TCCR2B & 0b11111000) | ((uint8_t) freq);
}
static void PWMfreq::set(MegaTimer3 freq){
  TCCR3B = (TCCR3B & 0b11111000) | ((uint8_t) freq);
}
static void PWMfreq::set(MegaTimer4 freq){
  TCCR4B = (TCCR4B & 0b11111000) | ((uint8_t) freq);
}
static void PWMfreq::set(MegaTimer5 freq){
  TCCR5B = (TCCR5B & 0b11111000) | ((uint8_t) freq);
}
#endif

#endif


// ==================================================
// PinControl
// ==================================================

PinControl::PinControl(uint8_t pin){
  this->pin = pin;
  pinMode(pin, OUTPUT);
  setLimits(0.0, 0.0);
}

PinControl::PinControl(uint8_t pin, float v1, float v2){
  this->pin = pin;
  pinMode(pin, OUTPUT);
  setLimits(v1, v2);
}

void PinControl::setLimits(float v1, float v2){
  this->v1 = v1;
  this->v2 = v2;
}

void PinControl::set(bool state){
  digitalWrite(pin, state ? HIGH : LOW);
}

void PinControl::pwm(uint8_t pwm){
  analogWrite(pin, pwm);
}

void PinControl::control(float value){
  pwm(remap(value, v1, v2, 0l, 255l, true));
}

#if defined(PIN_CONTROL_FEATURES)
void PinControl::feedback(float error){
  control(pid->evolve(error));
}

void PinControl::setPID(PID *pid){
  this->pid = pid;
}

PID* PinControl::getPID(){
  return this->pid;
}
#endif


// ==================================================
// PinMeasure
// ==================================================

PinMeasure::PinMeasure(uint8_t pin, bool pullup = false){
  this->pin = pin;
  pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
  setLimits(0.0, 0.0);
}

PinMeasure::PinMeasure(uint8_t pin, float v1, float v2, bool pullup = false){
  this->pin = pin;
  pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
  setLimits(v1, v2);
}

void PinMeasure::setLimits(float v1, float v2){
  this->v1 = v1;
  this->v2 = v2;
}

bool PinMeasure::state(){
  return digitalRead(pin) == HIGH;
}

uint16_t PinMeasure::value(){
  return analogRead(pin);
}

float PinMeasure::measure(){
  return remap((long) value(), 0l, 1023l, v1, v2);
}

#if defined(PIN_CONTROL_FEATURES)
float PinMeasure::filter(){
  return (fil != NULL) ? fil->evolve(measure()) : measure();
}

void PinMeasure::setFilter(Filter *filter){
  this->fil = filter;
}

Filter* PinMeasure::getFilter(){
  return this->fil;
}
#endif


// ==================================================
// Motor
// ==================================================

Motor::Motor(PinControl &INA, PinControl &INB, PinControl &PWM, PinMeasure &CHA, PinMeasure &CHB, PinMeasure &END) 
  : pin_INA(INA), pin_INB(INB), pin_PWM(PWM), pin_CHA(CHA), pin_CHB(CHB), pin_END(END){
  readEncoder();
}

Motor::~Motor() {}

void Motor::invertEncoder(bool invert){
  this->encoder_invert = invert;
}

long Motor::getEncoder(){
  return this->encoder;
}

void Motor::setEncoder(long value){
  this->encoder = value;
}

void Motor::readEncoder(){
  enc_A = pin_CHA.state();
  enc_B = pin_CHB.state();
}

void Motor::updateEncoder(){
  bool old_A = enc_A;
  readEncoder();
  if(old_A != enc_A){
    if(enc_B == enc_A) {
      encoder_invert ? encoder-- : encoder++;
    } else {
      encoder_invert ? encoder++ : encoder--;
    }
  }
}

void Motor::invertMotor(bool invert){
  this->motor_invert = invert;
}

void Motor::driveMotor(short spwm){
  OperatingMode mode = OperatingMode::BRAKE_GND;
  spwm = constrain(spwm, -255, 255);

  if(spwm > 0) {
    mode = motor_invert ? OperatingMode::SPIN_CCW : OperatingMode::SPIN_CW;
  } else if (spwm < 0) {
    mode = motor_invert ? OperatingMode::SPIN_CW : OperatingMode::SPIN_CCW;
  } else {
    mode = OperatingMode::BRAKE_GND;
  }
  
  switch(mode){
    case OperatingMode::BRAKE_GND:
      pin_INA.set(false);
      pin_INB.set(false);
      break;
    case OperatingMode::SPIN_CCW:
      pin_INA.set(false);
      pin_INB.set(true);
      break;
    case OperatingMode::SPIN_CW:
      pin_INA.set(true);
      pin_INB.set(false);
      break;
    case OperatingMode::BRAKE_VCC:
      pin_INA.set(true);
      pin_INB.set(true);
      break;
  }

  pin_PWM.pwm((uint8_t) abs(spwm));
}

bool Motor::isInEndStop(){
  return pin_END.state();
}


// ==================================================
// Robot
// ==================================================

Robot::Robot(PinControl &enable, PinControl &toggle, unsigned long ts_ms, uint8_t size, Motor **motors, float *encs_div)
  : pin_enable(enable), pin_toggle(toggle) {
  this->ts = ts_ms;
  this->motors = (Motor**) malloc(size * sizeof(Motor*));
  this->pids = (PID*) malloc(size * sizeof(PID));
  this->switches = malloc(size * sizeof(bool));
  this->motors_pwm = malloc(size * sizeof(short));
  this->encoders_rcv = malloc(size * sizeof(long));
  this->encoders_snd = malloc(size * sizeof(long));
  this->error_div = malloc(size * sizeof(float));
  
  this->size = size;
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
  
  timer.setup(ts_ms);
  update();
}

Robot::Robot(PinControl &enable, PinControl &toggle, unsigned long ts_ms, uint8_t size, Motor **motors) 
  : Robot(enable, toggle,ts_ms, size, motors, NULL) {}

Robot::Robot(PinControl &enable, PinControl &toggle, unsigned long ts_ms, uint8_t size) 
  : Robot(enable, toggle,ts_ms, size, NULL, NULL) {}

Robot::~Robot() {
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

int Robot::getSize(){
  return this->size;
}

Robot::Status Robot::getStatus(){
  return this->status;
}

void Robot::setStatus(Status status, bool reset = false){
  if(this->status != status || reset){
    resetPWMs();
    resetPIDs();
    this->status = status;
  }
}

Motor * Robot::getMotor(uint8_t index){
  return this->motors[index];
}

void Robot::setMotor(uint8_t index, Motor * motor){
  this->motors[index] = motor;
}

void Robot::setMotor(uint8_t index, Motor * motor, float enc_div){
  this->motors[index] = motor;
  this->error_div[index] = enc_div;
}

void Robot::setEncoderDivider(uint8_t index, float enc_div){
  this->error_div[index] = enc_div;
}

PID * Robot::getPID(uint8_t index){
  return &(this->pids[index]);
}

void Robot::initPIDs(float ts, float pole, float sat, bool bumpless){
  for(int i = 0; i < size; i++){
    getPID(i)->init(ts, pole, sat, bumpless);
  }
}

void Robot::setupPIDs(float kp, float ki, float kd){
  for(int i = 0; i < size; i++){
    getPID(i)->setup(kp, ki, kd);
  }
}

void Robot::resetPIDs(){
  for(int i = 0; i < size; i++){
    getPID(i)->reset();
  }
}

void Robot::updateEncoders(){
  for(int i = 0; i < size; i++){
    getMotor(i)->updateEncoder();
  }
}

void Robot::setEncoders(long *values){
  for(int i = 0; i < size; i++){
    setEncoder(i, values[i]);
  }  
}

void Robot::setEncoder(uint8_t index, long value){
  getMotor(index)->setEncoder(value);
}

void Robot::resetEncoders(){
  for(int i = 0; i < size; i++){
    setEncoder(i, 0);
  }
}


void Robot::setPWMs(short *pwms){
  for(int i = 0; i < size; i++){
    setPWM(i, pwms[i]);
  }
}

void Robot::setPWM(uint8_t index, short pwm){
  motors_pwm[index] = (short) constrain(pwm, -255, 255);
}

void Robot::resetPWMs(){
  for(int i = 0; i < size; i++){
    motors_pwm[i] = 0;
  }
}

void Robot::enableMotors(){
  setStatus(Status::Idle, true);
  pin_enable.set(true);
}

void Robot::disableMotors(){
  setStatus(Status::Idle, true);
  pin_enable.set(false);
}

Communication::Next Robot::peek(){
  return Communication::peek();
}

void Robot::rcvCtrl(){
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

void Robot::sndCtrl(){
  snd_ctrl->num = size;
  snd_ctrl->status = (unsigned char) status;

  for(int i = 0; i < size; i++) {
    snd_ctrl->switches[i] = switches[i];
    snd_ctrl->values[i] = min(getMotor(i)->getEncoder() - encoders_snd[i], 255);
    encoders_snd[i] += snd_ctrl->values[i];
  }
  
  Communication::snd(snd_ctrl);
}

void Robot::rcvSetup(){
  Communication::rcv(rcv_setup);
  
  setEncoderDivider(rcv_setup->num, rcv_setup->values[0]);
  getPID(rcv_setup->num)->reset();
  getPID(rcv_setup->num)->init((float) ts/1000.0, rcv_setup->values[4], rcv_setup->values[5], true);
  getPID(rcv_setup->num)->setup(rcv_setup->values[1], rcv_setup->values[2], rcv_setup->values[3]);

  setStatus(Status::Idle, true);
}

void Robot::sndSetup(){
  snd_setup->status = (unsigned char) Status::Setup;
  Communication::snd(snd_setup);
}

void Robot::update(){
  switch(status){
    case Status::Idle:
      resetPWMs();
      break;

    case Status::DAQ:
      break;

    case Status::PID:
      for(int i = 0; i < size; i++){
        float err = (float) (getMotor(i)->getEncoder() - encoders_rcv[i]) / ((error_div[i] == 0) ? 1.0 : error_div[i]);
        motors_pwm[i] = (short) constrain(getPID(i)->evolve(err), -255.0, 255.0);
      }
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

void Robot::actuate(){
  for(int i = 0; i < size; i++){
    getMotor(i)->driveMotor(motors_pwm[i]);
  }
}

void Robot::cycle(unsigned long time_ms){
  updateEncoders();
  Communication::Next next = peek();

  if(next == Communication::Next::Setup){
    rcvSetup();
    sndSetup();
    timer.reset(time_ms);
  } else if (next == Communication::Next::Error) {
    Serial.flush();
    setStatus(Robot::Status::Idle, true);
    timer.reset(time_ms);
  } else {
    if(getStatus() == Robot::Status::Idle && next == Communication::Next::Ctrl){
      pin_toggle.set(true);
      rcvCtrl();
      update();
      actuate();
      pin_toggle.set(false);
      timer.reset(time_ms);
      if(getStatus() == Robot::Status::Idle) {
        sndCtrl();
      }
    } else if (getStatus() != Robot::Status::Idle) {
      if (timer.check(time_ms)){
        pin_toggle.set(true);
        sndCtrl();
        rcvCtrl();
        update();
        actuate();
        pin_toggle.set(false);
        if(getStatus() == Robot::Status::Idle){
          sndCtrl();
        }
      }
    }
  } 
}