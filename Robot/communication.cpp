#include "communication.h"


// ==================================================
// Communication
// ==================================================

#if defined(MEGA)
static void Communication::channel(uint8_t index){
  switch(index){
    case 0:
      hwserial = &Serial;
      break;
    case 1:
      hwserial = &Serial1;
      break;
    case 2:
      hwserial = &Serial2;
      break;
    case 3:
      hwserial = &Serial3;
      break;
    default:
      hwserial = &Serial;
      break;
  }
}
#else
static void Communication::channel(uint8_t index){
  switch(index){
    case 0:
      hwserial = &Serial;
      break;
    default:
      hwserial = &Serial;
      break;
  }
}
#endif

static void Communication::flush(){
  while(hwserial->available()) {
    hwserial->read();
  }
  hwserial->flush();
}

static bool Communication::peek(Communication::Header *hdr, Timer *timeout_us = NULL){
  if(hdr == NULL) return false;
  int res = -1;
  if(timeout_us == NULL) {
    res = hwserial->peek();
  } else {
    while(res == -1){
      if(timeout_us->check(micros())) break;
      res = hwserial->peek();
    } 
  }
  if(res == -1) return false;
  hdr->parse(res);
  #if defined(DEBUG_COMMUNICATION)
  Serial.print("peek: ");
  Serial.print((uint8_t) hdr->getCode());
  Serial.println();
  #endif
  return true;
}

static bool Communication::rcv(Communication::Message *msg, Timer *timeout_us = NULL){
  if(msg == NULL) return false;
  uint8_t buffer[msg->size()];
  while(hwserial->available() < msg->size()) if(timeout_us != NULL && timeout_us->check(micros())) return false;
  if(hwserial->readBytes(buffer, msg->size()) != msg->size()) return false;
  bool valid = msg->from(buffer);
  #if defined(DEBUG_COMMUNICATION)
  Serial.print("rcv: ");
  Serial.print((uint8_t) msg->getCode());
  Serial.print(" ");
  Serial.print(msg->size());
  Serial.print(" ");
  Serial.print(hwserial->available());
  Serial.print(" |");
  for(int i = 0; i < msg->size(); i++){
    char hhex, lhex;
    byteToHex(buffer[i], hhex, lhex);
    Serial.print(" ");
    Serial.print(hhex);
    Serial.print(lhex);
  }
  Serial.println();
  #endif
  return valid;
}

static bool Communication::snd(Communication::Message *msg){
  if(msg == NULL) return false;
  uint8_t buffer[msg->size()];
  msg->fill(buffer);
  bool valid = hwserial->write(buffer, msg->size()) == msg->size();
  #if defined(DEBUG_COMMUNICATION)
  Serial.print("snd: ");
  Serial.print((uint8_t) msg->getCode());
  Serial.print(" ");
  Serial.print(msg->size());
  Serial.print(" ");
  Serial.print(hwserial->availableForWrite());
  Serial.print(" |");
  for(int i = 0; i < msg->size(); i++){
    char hhex, lhex;
    byteToHex(buffer[i], hhex, lhex);
    Serial.print(" ");
    Serial.print(hhex);
    Serial.print(lhex);
  }
  Serial.println();
  #endif
  return valid;
}

static bool Communication::convert(uint8_t value, Communication::Code &code){
  switch(value){
    case (uint8_t) Code::IDLE:
      code = Code::IDLE;
      return true;
    case (uint8_t) Code::PWM:
      code = Code::PWM;
      return true;
    case (uint8_t) Code::REF:
      code = Code::REF;
      return true;
    case (uint8_t) Code::ROBOT:
      code = Code::ROBOT;
      return true;
    case (uint8_t) Code::MOTOR:
      code = Code::MOTOR;
      return true;
    case (uint8_t) Code::PID:
      code = Code::PID;
      return true;
    case (uint8_t) Code::ACKC:
      code = Code::ACKC;
      return true;
    case (uint8_t) Code::ACKS:
      code = Code::ACKS;
      return true;
    case (uint8_t) Code::ERROR:
      code = Code::ERROR;
      return true;
    default:
      code = Code::ERROR;
      return false;
  }
}

static bool Communication::isCtrl(Code code){
  uint8_t value = (uint8_t) code;
  return ((value & 0b10000) == 0) && ((value & 0b01000) == 0) && ((value & 0b00100) == 0);
}

static bool Communication::isSetup(Code code){
  uint8_t value = (uint8_t) code;
  return ((value & 0b10000) != 0) && ((value & 0b01000) == 0) && ((value & 0b00100) == 0);
}

static bool Communication::isAck(Code code){
  uint8_t value = (uint8_t) code;
  return ((value & 0b10000) != 0) && ((value & 0b01000) != 0) && ((value & 0b00100) == 0);
}

static bool Communication::isError(Code code){
  uint8_t value = (uint8_t) code;
  return ((value & 0b10000) != 0) && ((value & 0b01000) != 0) && ((value & 0b00100) != 0);
}

// ===== Header =====

Communication::Header::Header(Communication::Code code, uint8_t num) {
  setCode(code);
  setNum(num);
}

Communication::Code Communication::Header::getCode(){
  return this->code;
}

uint8_t Communication::Header::getNum(){
  return this->num;
}

bool Communication::Header::setCode(Communication::Code code){
  this->code = code;
  return true;
}

bool Communication::Header::setNum(uint8_t num){
  if(num < 8) {
    this->num = num;
    return true;
  } else {
    return false;
  }
}

bool Communication::Header::parse(uint8_t byte){
  Code code;
  if(!convert((byte & 0b11111000) >> 3, code)) return false;
  setCode(code);
  setNum(byte & 0b00000111);
  return true;
}

uint8_t Communication::Header::byte(){
  return (((uint8_t) this->code) << 3) | num;
}

uint8_t Communication::Header::size(){
  return 1;
}

uint8_t Communication::Header::from(uint8_t *buffer){
  return parse(buffer[0]);
}

uint8_t Communication::Header::fill(uint8_t *buffer){
  buffer[0] = byte();
  return 1;
}

// ===== Message =====

Communication::Message::Message(Code code, uint8_t num){
  header.setCode(code);
  header.setNum(num);
}

Communication::Code Communication::Message::getCode(){
  return header.getCode();
}

uint8_t Communication::Message::getNum(){
  return header.getNum();
}

bool Communication::Message::setNum(uint8_t num){
  return header.setNum(num);
}

uint8_t Communication::Message::size(){
  return size_header() + size_payload();
}

uint8_t Communication::Message::from(uint8_t *buffer){
  if(from_header(buffer) != size_header()) return 0;
  if(from_payload(buffer) != size_payload()) return 0;
  return size();
}

uint8_t Communication::Message::fill(uint8_t *buffer){
  if(fill_header(buffer) != size_header()) return 0;
  if(fill_payload(buffer) != size_payload()) return 0;
  return size();
}

uint8_t Communication::Message::size_payload(){
  return 0;
}

uint8_t Communication::Message::from_payload(uint8_t *buffer){
  return size_payload();
}

uint8_t Communication::Message::fill_payload(uint8_t *buffer){
  return size_payload();
}

uint8_t Communication::Message::size_header(){
  return header.size();
}

uint8_t Communication::Message::from_header(uint8_t *buffer){
  Code code;
  if(!convert((buffer[0] & 0b11111000) >> 3, code)) return 0;
  if(getCode() != code) return 0;
  setNum(buffer[0] & 0b00000111);
  return size_header();
}

uint8_t Communication::Message::fill_header(uint8_t *buffer){
  buffer[0] = (((uint8_t) getCode()) << 3) | getNum();
  return size_header();
}

// ===== MsgIDLE =====

uint8_t Communication::MsgIDLE::getCount(){
  return getNum() + 1;
}

bool Communication::MsgIDLE::setCount(uint8_t count){
  return setNum(count - 1);
}

uint8_t Communication::MsgIDLE::size_payload(){
  return 0;
}

uint8_t Communication::MsgIDLE::from_payload(uint8_t *buffer){
  return size_payload();
}

uint8_t Communication::MsgIDLE::fill_payload(uint8_t *buffer){
  return size_payload();
}

// ===== MsgPWM =====

uint8_t Communication::MsgPWM::getCount(){
  return getNum() + 1;
}

int16_t Communication::MsgPWM::getPwm(uint8_t index){
  if(index < getCount()){
    return pwms[index];
  } else {
    return 0;
  }
}

bool Communication::MsgPWM::setCount(uint8_t count){
  return setNum(count - 1);
}

bool Communication::MsgPWM::setPwm(uint8_t index, int16_t value){
  if(index < getCount()){
    pwms[index] = min(max(value, -255), 255);
    return true;
  } else {
    return false;
  }
}

uint8_t Communication::MsgPWM::size_payload(){
  return 1 + getCount();
}

uint8_t Communication::MsgPWM::from_payload(uint8_t *buffer){
  for(uint8_t i = 0; i < getCount(); i++){
    setPwm(i, ((buffer[1] & ((0b00000001) << i)) ? -1 : +1) * ((int16_t) buffer[2+i]));
  }
  return size_payload();
}

uint8_t Communication::MsgPWM::fill_payload(uint8_t *buffer){
  buffer[1] = 0b00000000;
  for(uint8_t i = 0; i < getCount(); i++){
    buffer[1] = buffer[1] | ((getPwm(i) < 0) << i);
    buffer[2+i] = abs(getPwm(i));
  }
  return size_payload();
}

// ===== MsgREF =====

uint8_t Communication::MsgREF::getCount(){
  return getNum() + 1;
}

int16_t Communication::MsgREF::getDeltaEnc(uint8_t index){
  if(index < getCount()){
    return deltas[index];
  } else {
    return 0;
  }
}

bool Communication::MsgREF::setCount(uint8_t count){
  return setNum(count - 1);
}

bool Communication::MsgREF::setDeltaEnc(uint8_t index, int16_t value){
  if(index < getCount()){
    deltas[index] = min(max(value, -255), 255);
    return true;
  } else {
    return false;
  }
}

uint8_t Communication::MsgREF::size_payload(){
  return 1 + getCount();
}

uint8_t Communication::MsgREF::from_payload(uint8_t *buffer){
  for(uint8_t i = 0; i < getCount(); i++){
    setDeltaEnc(i, ((buffer[1] & (1 << i)) ? -1 : +1) * ((int16_t) buffer[2+i]));
  }
  return size_payload();
}

uint8_t Communication::MsgREF::fill_payload(uint8_t *buffer){
  buffer[1] = 0b00000000;
  for(uint8_t i = 0; i < getCount(); i++){
    buffer[1] = buffer[1] | ((getDeltaEnc(i) < 0) << i);
    buffer[2+i] = abs(getDeltaEnc(i));
  }
  return size_payload();
}

// ===== MsgROBOT =====

uint8_t Communication::MsgROBOT::getCount(){
  return getNum() + 1;
}

uint32_t Communication::MsgROBOT::getTimeSampling(){
  return timesampling_us;
}

bool Communication::MsgROBOT::setCount(uint8_t count){
  return setNum(count - 1);
}

bool Communication::MsgROBOT::setTimeSampling(uint32_t value){
  timesampling_us = value;
  return true;
}

uint8_t Communication::MsgROBOT::size_payload(){
  return 4;
}

uint8_t Communication::MsgROBOT::from_payload(uint8_t *buffer){
  uint32_t ts;
  memcpy((void *) &ts, (void *) (buffer+1), 4);
  setTimeSampling(ts);
  return size_payload();
}

uint8_t Communication::MsgROBOT::fill_payload(uint8_t *buffer){
  uint32_t ts;
  ts = getTimeSampling();
  memcpy((void *) (buffer+1), (void *) &ts, 4);
  return size_payload();
}

// ===== MsgMOTOR =====

uint8_t Communication::MsgMOTOR::getIndex(){
  return getNum();
}

bool Communication::MsgMOTOR::getChangeEncoder(){
  return flags & (1u << 0);
}

bool Communication::MsgMOTOR::getInvertSpinDir(){
  return flags & (1u << 1);
}

bool Communication::MsgMOTOR::getChangeSpinDir(){
  return flags & (1u << 2);
}

int8_t Communication::MsgMOTOR::getSpinDirection(){
  return getChangeSpinDir() ? (getInvertSpinDir() ? -1 : 1) : 0;
}

bool Communication::MsgMOTOR::getInvertEncDir(){
  return flags & (1u << 3);
}

bool Communication::MsgMOTOR::getChangeEncDir(){
  return flags & (1u << 4);
}

int8_t Communication::MsgMOTOR::getEncDirection(){
  return getChangeEncDir() ? (getInvertEncDir() ? -1 : 1) : 0;
}

int32_t Communication::MsgMOTOR::getEncoderValue(){
  return encoder;
}

bool Communication::MsgMOTOR::setIndex(uint8_t index){
  return setNum(index);
}

bool Communication::MsgMOTOR::setChangeEncoder(bool value){
  flags = flags | (value << 0);
  return true;
}

bool Communication::MsgMOTOR::setInvertSpinDir(bool value){
  flags = flags | (value << 1);
  return true;
}

bool Communication::MsgMOTOR::setChangeSpinDir(bool value){
  flags = flags | (value << 2);
  return true;
}

bool Communication::MsgMOTOR::setSpinDirection(int8_t dir){
  setInvertSpinDir(dir < 0);
  setChangeSpinDir(dir != 0);
  return true;
}

bool Communication::MsgMOTOR::setInvertEncDir(bool value){
  flags = flags | (value << 3);
  return true;
}

bool Communication::MsgMOTOR::setChangeEncDir(bool value){
  flags = flags | (value << 4);
  return true;
}

bool Communication::MsgMOTOR::setEncDirection(int8_t dir){
  setInvertEncDir(dir < 0);
  setChangeEncDir(dir != 0);
  return true;
}

bool Communication::MsgMOTOR::setEncoderValue(int32_t value){
  encoder = value;
  return true;
}

uint8_t Communication::MsgMOTOR::size_payload(){
  return 5;
}

uint8_t Communication::MsgMOTOR::from_payload(uint8_t *buffer){
  bool ec;
  int16_t sd;
  int16_t ed;
  int32_t ev;
  ec = (buffer[1] & (1 << 0));
  sd = (buffer[1] & (1 << 2)) ? ((buffer[1] & (1 << 1)) ? -1 : +1) : 0;
  ed = (buffer[1] & (1 << 3)) ? ((buffer[1] & (1 << 4)) ? -1 : +1) : 0;
  memcpy((void *) &ev , (void *) (buffer + 1), 4);
  setChangeEncoder(ec);
  setSpinDirection(sd);
  setEncDirection(ed);
  setEncoderValue(ev);
  return size_payload();
}

uint8_t Communication::MsgMOTOR::fill_payload(uint8_t *buffer){
  bool ec;
  int16_t sd;
  int16_t ed;
  int32_t ev;
  ec = getChangeEncoder();
  sd = getSpinDirection();
  ed = getEncDirection();
  ev = getEncoderValue(); 
  buffer[1] = ((ed == 0) << 4) | ((ed == -1) << 3) | ((sd == 0) << 2) | ((sd == -1) << 1) | (ec << 0);
  memcpy((void *) (buffer + 1), (void *) &ev , 4);
  return size_payload();
}

// ===== MsgPID =====

uint8_t Communication::MsgPID::getIndex(){
  return getNum();
}

float Communication::MsgPID::getPidDiv(){
  return div;
}

float Communication::MsgPID::getPidKp(){
  return kp;
}

float Communication::MsgPID::getPidKi(){
  return ki;
}

float Communication::MsgPID::getPidKd(){
  return kd;
}

float Communication::MsgPID::getPidSat(){
  return sat;
}

float Communication::MsgPID::getPidPole(){
  return pole;
}

bool Communication::MsgPID::setIndex(uint8_t index){
  return setNum(index);
}

bool Communication::MsgPID::setPidDiv(float value){
  div = value;
  return true;
}

bool Communication::MsgPID::setPidKp(float value){
  kp = value;
  return true;
}

bool Communication::MsgPID::setPidKi(float value){
  ki = value;
  return true;
}

bool Communication::MsgPID::setPidKd(float value){
  kd = value;
  return true;
}

bool Communication::MsgPID::setPidSat(float value){
  sat = value;
  return true;
}

bool Communication::MsgPID::setPidPole(float value){
  pole = value;
  return true;
}

uint8_t Communication::MsgPID::size_payload(){
  return 24;
}

uint8_t Communication::MsgPID::from_payload(uint8_t *buffer){
  float div;
  float kp;
  float ki;
  float kd;
  float sat;
  float pole; 
  memcpy((void *) &div , (void *) (buffer+ 1), 4);
  memcpy((void *) &kp  , (void *) (buffer+ 5), 4);
  memcpy((void *) &ki  , (void *) (buffer+ 9), 4);
  memcpy((void *) &kd  , (void *) (buffer+13), 4);
  memcpy((void *) &sat , (void *) (buffer+17), 4);
  memcpy((void *) &pole, (void *) (buffer+21), 4);
  setPidDiv(div);
  setPidKp(kp);
  setPidKi(ki);
  setPidKd(kd);
  setPidSat(sat);
  setPidPole(pole);
  return size_payload();
}

uint8_t Communication::MsgPID::fill_payload(uint8_t *buffer){
  float div;
  float kp;
  float ki;
  float kd;
  float sat;
  float pole; 
  div = getPidDiv();
  kp = getPidKp();
  ki = getPidKi();
  kd = getPidKd();
  sat = getPidSat();
  pole = getPidPole();
  memcpy((void *) (buffer+ 1), (void *) &div , 4);
  memcpy((void *) (buffer+ 5), (void *) &kp  , 4);
  memcpy((void *) (buffer+ 9), (void *) &ki  , 4);
  memcpy((void *) (buffer+13), (void *) &kd  , 4);
  memcpy((void *) (buffer+17), (void *) &sat , 4);
  memcpy((void *) (buffer+21), (void *) &pole, 4);
  return size_payload();
}

// ===== MsgACKC =====

uint8_t Communication::MsgACKC::getCount(){
  return getNum() + 1;
}

bool Communication::MsgACKC::getEndStop(uint8_t index){
  if(index < getCount()){
    return endstops & (1 << index);
  } else {
    return false;
  }
}

int16_t Communication::MsgACKC::getDeltaEnc(uint8_t index){
  if(index < getCount()){
    return deltas[index];
  } else {
    return 0;
  }
}

bool Communication::MsgACKC::setCount(uint8_t count){
  return setNum(count - 1);
}

bool Communication::MsgACKC::setEndStop(uint8_t index, bool value){
  if(index < getCount()){
    endstops = endstops | (value << index);
    return true;
  } else {
    return false;
  }
}

bool Communication::MsgACKC::setDeltaEnc(uint8_t index, int16_t value){
  if(index < getCount()){
    deltas[index] = min(max(value, -255), 255);
    return true;
  } else {
    return false;
  }
}

uint8_t Communication::MsgACKC::size_payload(){
  return 2 + getCount();
}

uint8_t Communication::MsgACKC::from_payload(uint8_t *buffer){
  for(uint8_t i = 0; i < getCount(); i++){
    setEndStop(i, buffer[1] & (1 << i));
    setDeltaEnc(i, ((buffer[2] & (1 << i)) ? -1 : +1) * ((int16_t) buffer[3+i]));
  }
  return size_payload();
}

uint8_t Communication::MsgACKC::fill_payload(uint8_t *buffer){
  buffer[1] = 0b00000000;
  buffer[2] = 0b00000000;
  for(uint8_t i = 0; i < getCount(); i++){
    buffer[1] = buffer[1] | (getEndStop(i) << i);
    buffer[2] = buffer[2] | ((getDeltaEnc(i) < 0) << i);
    buffer[3+i] = (uint8_t) abs(getDeltaEnc(i));
  }
  return size_payload();
}

// ===== MsgACKS =====

uint8_t Communication::MsgACKS::getCount(){
  return getNum() + 1;
}

uint8_t Communication::MsgACKS::getIndex(){
  return getNum();
}

bool Communication::MsgACKS::setCount(uint8_t count){
  return setNum(count - 1);
}

bool Communication::MsgACKS::setIndex(uint8_t index){
  return setNum(index);
}

uint8_t Communication::MsgACKS::size_payload(){
  return 0;
}

uint8_t Communication::MsgACKS::from_payload(uint8_t *buffer){
  return size_payload();
}

uint8_t Communication::MsgACKS::fill_payload(uint8_t *buffer){
  return size_payload();
}

// ===== MsgERROR =====

uint8_t Communication::MsgERROR::getCount(){
  return getNum() + 1;
}

uint8_t Communication::MsgERROR::getIndex(){
  return getNum();
}

bool Communication::MsgERROR::setCount(uint8_t count){
  return setNum(count - 1);
}

bool Communication::MsgERROR::setIndex(uint8_t index){
  return setNum(index);
}

uint8_t Communication::MsgERROR::size_payload(){
  return 0;
}

uint8_t Communication::MsgERROR::from_payload(uint8_t *buffer){
  return size_payload();
}

uint8_t Communication::MsgERROR::fill_payload(uint8_t *buffer){
  return size_payload();
}


// ==================================================
// RobotComm
// ==================================================

RobotComm::RobotComm(Robot &robot, uint8_t channel) :
  robot(robot) 
{
  this->channel = channel;
  this->timer.setup(robot.getTimeSampling());
  this->timeout.setup(robot.getTimeSampling());
  this->encoders_rcv = malloc(robot.getSize() * sizeof(long));
  this->encoders_snd = malloc(robot.getSize() * sizeof(long));

  for(int i = 0; i < robot.getSize(); i++) {
    this->encoders_rcv[i] = 0;
    this->encoders_snd[i] = 0;
  }
}

RobotComm::~RobotComm(){
  free(this->encoders_rcv);
  free(this->encoders_snd);
}

void RobotComm::cycle(uint32_t time_us){
  robot.update();

  Communication::channel(channel);
  Communication::Header header;

  bool res = Communication::peek(&header);
  
  if(res){
    timeout.reset(time_us);
    Communication::Code code = header.getCode();
    
    if(Communication::isCtrl(code)) {
      if(robot.getStatus() == Robot::Status::Idle){
        timer.reset(time_us);
      }
      
      if(robot.getStatus() == Robot::Status::Idle || timer.check(time_us)) {
        #if defined(DEBUG_COMMUNICATION)
        Serial.print("Receiving Control ");
        #endif
        
        switch(code){
          case Communication::Code::IDLE:
            {
              #if defined(DEBUG_COMMUNICATION)
              Serial.println("IDLE");
              #endif
              Communication::MsgIDLE msg_idle;
              msg_idle.setCount(robot.getSize());
              res = Communication::rcv(&msg_idle, &timeout);
              res = res && msg_idle.getCount() == robot.getSize();
              #if defined(DEBUG_COMMUNICATION)
              Serial.print("  operation: ");
              Serial.println(res ? "Succeded" : "Failed");
              #endif
              if(!res) break;
              robot.setStatus(Robot::Status::Idle);
              #if defined(DEBUG_COMMUNICATION)
              Serial.print("  count: ");
              Serial.println(msg_idle.getCount());
              #endif
              break;
            }

          case Communication::Code::PWM:
            {
              #if defined(DEBUG_COMMUNICATION)
              Serial.println("PWM");
              #endif
              Communication::MsgPWM msg_pwm;
              msg_pwm.setCount(robot.getSize());
              res = Communication::rcv(&msg_pwm, &timeout);
              res = res && msg_pwm.getCount() == robot.getSize();
              #if defined(DEBUG_COMMUNICATION)
              Serial.print("  operation: ");
              Serial.println(res ? "Succeded" : "Failed");
              #endif
              if(!res) break;
              robot.setStatus(Robot::Status::Pwm);
              for(uint8_t i = 0; i < robot.getSize(); i++) {
                robot.setPwm(i, msg_pwm.getPwm(i));
              }
              #if defined(DEBUG_COMMUNICATION)
              Serial.print("  count: ");
              Serial.println(msg_pwm.getCount());
              for(int i = 0; i < msg_pwm.getCount(); i++) {
                Serial.print("  pmw ");
                Serial.print(i);
                Serial.print(": ");
                Serial.println(msg_pwm.getPwm(i));
              }
              #endif
              break;
            }

          case Communication::Code::REF:
            {
              #if defined(DEBUG_COMMUNICATION)
              Serial.println("REF");
              #endif
              Communication::MsgREF msg_ref;
              msg_ref.setCount(robot.getSize());
              res = Communication::rcv(&msg_ref, &timeout);
              res = res && msg_ref.getCount() == robot.getSize();
              #if defined(DEBUG_COMMUNICATION)
              Serial.print("  operation: ");
              Serial.println(res ? "Succeded" : "Failed");
              #endif
              if(!res) break;
              robot.setStatus(Robot::Status::Ref);
              for(uint8_t i = 0; i < robot.getSize(); i++) {
                encoders_rcv[i] += msg_ref.getDeltaEnc(i);
                robot.setTarget(i, encoders_rcv[i]);
              }
              #if defined(DEBUG_COMMUNICATION)
              Serial.print("  count: ");
              Serial.println(msg_ref.getCount());
              for(int i = 0; i < msg_ref.getCount(); i++) {
                Serial.print("  denc ");
                Serial.print(i);
                Serial.print(": ");
                Serial.println(msg_ref.getDeltaEnc(i));
              }
              #endif
              break;
            }

          default:
            #if defined(DEBUG_COMMUNICATION)
            Serial.println("Unexpected");
            #endif
            res = false;
            break;
        }
        
        if(res){
          robot.compute();
          robot.actuate();

          Communication::MsgACKC msg_ackc;
          msg_ackc.setCount(robot.getSize());
          for(uint8_t i = 0; i < robot.getSize(); i++){
            msg_ackc.setEndStop(i, robot.getEndStop(i));
            msg_ackc.setDeltaEnc(i, robot.getEncoder(i) - encoders_snd[i]);
          }

          res = Communication::snd(&msg_ackc);

          #if defined(DEBUG_COMMUNICATION)
          Serial.println("Sending Control ACKC");
          Serial.print("  operation: ");
          Serial.println(res ? "Succeded" : "Failed");
          Serial.print("  count: ");
          Serial.println(msg_ackc.getCount());
          Serial.print("  endstops:");
          for(uint8_t i = 0; i < robot.getSize(); i++){
            Serial.print(msg_ackc.getEndStop(i) ? " 1" : " 0");
          }
          Serial.println();
          for(uint8_t i = 0; i < robot.getSize(); i++){
            Serial.print("  denc ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(msg_ackc.getDeltaEnc(i));
          }
          #endif

          if(res) {
            for(uint8_t i = 0; i < robot.getSize(); i++){
              encoders_snd[i] += msg_ackc.getDeltaEnc(i);
            }
          }

          #if defined(DEBUG_COMMUNICATION)
          Serial.print("Encoder rob:");
          for(uint8_t i = 0; i < robot.getSize(); i++){
            Serial.print(" ");
            Serial.print(robot.getEncoder(i));
          }
          Serial.println();
          #endif

          #if defined(DEBUG_COMMUNICATION)
          Serial.print("Encoder rcv:");
          for(uint8_t i = 0; i < robot.getSize(); i++){
            Serial.print(" ");
            Serial.print(encoders_snd[i]);
          }
          Serial.println();
          #endif

          #if defined(DEBUG_COMMUNICATION)
          Serial.print("Encoder snd:");
          for(uint8_t i = 0; i < robot.getSize(); i++){
            Serial.print(" ");
            Serial.print(encoders_snd[i]);
          }
          Serial.println();
          #endif
        }
      }
    } 
    
    else if(Communication::isSetup(code)) {
      robot.setStatus(Robot::Status::Idle, true);
      Communication::MsgACKS msg_acks;

      #if defined(DEBUG_COMMUNICATION)
      Serial.print("Receiving Setup ");
      #endif

      switch(code){
        case Communication::Code::ROBOT:
          {
            #if defined(DEBUG_COMMUNICATION)
            Serial.println("ROBOT");
            #endif
            Communication::MsgROBOT msg_robot;
            res = Communication::rcv(&msg_robot, &timeout);
            res = res && msg_robot.getCount() == robot.getSize();
            #if defined(DEBUG_COMMUNICATION)
            Serial.print("  operation: ");
            Serial.println(res ? "Succeded" : "Failed");
            #endif
            if(!res) break;
            robot.setTimeSampling(msg_robot.getTimeSampling());
            timer.setup(msg_robot.getTimeSampling());
            timeout.setup(msg_robot.getTimeSampling());
            #if defined(DEBUG_COMMUNICATION)
            Serial.print("  count: ");
            Serial.println(msg_robot.getCount());
            Serial.print("  time-sampling: ");
            Serial.println(msg_robot.getTimeSampling());
            #endif
            msg_acks.setCount(robot.getSize());
            break;
          }

        case Communication::Code::MOTOR:
          {
            #if defined(DEBUG_COMMUNICATION)
            Serial.println("MOTOR");
            #endif
            Communication::MsgMOTOR msg_motor;
            res = Communication::rcv(&msg_motor, &timeout);
            res = res && msg_motor.getIndex() < robot.getSize();
            #if defined(DEBUG_COMMUNICATION)
            Serial.print("  operation: ");
            Serial.println(res ? "Succeded" : "Failed");
            #endif
            if(!res) break;
            if(msg_motor.getChangeEncoder()) robot.setEncoder(msg_motor.getIndex(), msg_motor.getEncoderValue());
            if(msg_motor.getChangeSpinDir()) robot.invertMotor(msg_motor.getIndex(), msg_motor.getInvertSpinDir());
            if(msg_motor.getChangeEncDir()) robot.invertEncoder(msg_motor.getIndex(), msg_motor.getInvertEncDir());
            #if defined(DEBUG_COMMUNICATION)
            Serial.print("  index: ");
            Serial.println(msg_motor.getIndex());
            Serial.print("  encoder: ");
            Serial.print(msg_motor.getChangeEncoder() ? "1 " : "0 ");
            Serial.println(msg_motor.getEncoderValue());
            Serial.print("  spin dir: ");
            Serial.println(msg_motor.getSpinDirection());
            Serial.print("  enc dir: ");
            Serial.println(msg_motor.getEncDirection());
            #endif
            msg_acks.setIndex(msg_motor.getIndex());
            break;
          }

        case Communication::Code::PID:
          {
            #if defined(DEBUG_COMMUNICATION)
            Serial.println("PID");
            #endif
            Communication::MsgPID msg_pid;
            res = Communication::rcv(&msg_pid, &timeout);
            res = res && msg_pid.getIndex() < robot.getSize();
            #if defined(DEBUG_COMMUNICATION)
            Serial.print("  operation: ");
            Serial.println(res ? "Succeded" : "Failed");
            #endif
            if(!res) break;
            robot.initPID(msg_pid.getIndex(), msg_pid.getPidSat(),  msg_pid.getPidPole());
            robot.setupPID(msg_pid.getIndex(), msg_pid.getPidDiv(), msg_pid.getPidKp(), msg_pid.getPidKi(), msg_pid.getPidKd());
            robot.resetPID(msg_pid.getIndex());
            #if defined(DEBUG_COMMUNICATION)
            Serial.print("  index: ");
            Serial.println(msg_pid.getIndex());
            Serial.print("  div: ");
            Serial.print(msg_pid.getPidDiv(), 3);
            Serial.print("  kp: ");
            Serial.print(msg_pid.getPidKp(), 3);
            Serial.print("  ki: ");
            Serial.print(msg_pid.getPidKi(), 3);
            Serial.print("  kd: ");
            Serial.print(msg_pid.getPidKd(), 3);
            Serial.print("  sat: ");
            Serial.print(msg_pid.getPidSat(), 3);
            Serial.print("  pole: ");
            Serial.print(msg_pid.getPidPole(), 3);
            #endif
            msg_acks.setIndex(msg_pid.getIndex());
            break;
          }

        default:
          #if defined(DEBUG_COMMUNICATION)
          Serial.println("Unexpected");
          #endif
          res = false;
          break;
      }
      
      if(res){
        res = Communication::snd(&msg_acks);

        #if defined(DEBUG_COMMUNICATION)
        Serial.println("Sending Setup ACKS");
        Serial.print("  operation: ");
        Serial.println(res ? "Succeded" : "Failed");
        Serial.print("  count/index: ");
        Serial.print(msg_acks.getCount());
        Serial.print("/");
        Serial.println(msg_acks.getIndex());
        #endif
      }
    }

    else {
      res = false;

      #if defined(DEBUG_COMMUNICATION)
      Serial.println("Peeking Unexpected message");
      #endif
    }

    if(!res){
      robot.setStatus(Robot::Status::Idle, true);
      Communication::flush();
      Communication::MsgERROR msg_error;
      msg_error.setCount(robot.getSize());

      res = Communication::snd(&msg_error);

      #if defined(DEBUG_COMMUNICATION)
      Serial.println("Sending Error ERROR");
      Serial.print("  operation: ");
      Serial.println(res ? "Succeded" : "Failed");
      Serial.print("  count: ");
      Serial.println(msg_error.getCount());
      #endif
    }
  }
}

void RobotComm::cycle(){
  cycle(micros());
}