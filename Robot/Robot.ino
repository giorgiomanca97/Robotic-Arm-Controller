// CODE for 1-8 DoF Robot control board


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

// DC Motors PINs
#define MOTORS_EN   38      // Motors enabler

#define MOTOR_1_INA 22      // Motor 1 spin direction
#define MOTOR_1_INB 23      // Motor 1 spin direction
#define MOTOR_1_PWM 5       // Motor 1 spin pwm
#define MOTOR_1_CHA A8      // Motor 1 encoder A channel
#define MOTOR_1_CHB A14     // Motor 1 encoder B channel
#define MOTOR_1_END 37      // Motor 1 endstop switch

#define MOTOR_2_INA 24      // Motor 2 spin direction
#define MOTOR_2_INB 25      // Motor 2 spin direction
#define MOTOR_2_PWM 2       // Motor 2 spin pwm
#define MOTOR_2_CHA A9      // Motor 2 encoder A channel
#define MOTOR_2_CHB A15     // Motor 2 encoder B channel
#define MOTOR_2_END 36      // Motor 2 endstop switch

#define MOTOR_3_INA 26      // Motor 3 spin direction
#define MOTOR_3_INB 27      // Motor 3 spin direction
#define MOTOR_3_PWM 3       // Motor 3 spin pwm
#define MOTOR_3_CHA A10     // Motor 3 encoder A channel
#define MOTOR_3_CHB 10      // Motor 3 encoder B channel
#define MOTOR_3_END 35      // Motor 3 endstop switch

#define MOTOR_4_INA 28      // Motor 4 spin direction
#define MOTOR_4_INB 29      // Motor 4 spin direction
#define MOTOR_4_PWM 6       // Motor 4 spin pwm
#define MOTOR_4_CHA A11     // Motor 4 encoder A channel
#define MOTOR_4_CHB 11      // Motor 4 encoder B channel
#define MOTOR_4_END 34      // Motor 4 endstop switch

#define MOTOR_5_INA 49      // Motor 5 spin direction
#define MOTOR_5_INB 48      // Motor 5 spin direction
#define MOTOR_5_PWM 7       // Motor 5 spin pwm
#define MOTOR_5_CHA A12     // Motor 5 encoder A channel
#define MOTOR_5_CHB 12      // Motor 5 encoder B channel
#define MOTOR_5_END 33      // Motor 5 endstop switch

#define MOTOR_6_INA 47      // Motor 6 spin direction
#define MOTOR_6_INB 46      // Motor 6 spin direction
#define MOTOR_6_PWM 8       // Motor 6 spin pwm
#define MOTOR_6_CHA A13     // Motor 6 encoder A channel
#define MOTOR_6_CHB 13      // Motor 6 encoder B channel
#define MOTOR_6_END 32      // Motor 6 endstop switch

// Other pins
#define PIN_TOGGLE  52

// Control parameters
#define BAUDRATE    115200  // Serial baudrate
#define TS          10      // Control time sampling

#define ENC_1_DIV   1000.0  // Motor 1 divider for encorder error in PID mode
#define ENC_2_DIV   1000.0  // Motor 2 divider for encorder error in PID mode
#define ENC_3_DIV   1000.0  // Motor 3 divider for encorder error in PID mode
#define ENC_4_DIV   1000.0  // Motor 4 divider for encorder error in PID mode
#define ENC_5_DIV   1000.0  // Motor 5 divider for encorder error in PID mode
#define ENC_6_DIV   1000.0  // Motor 6 divider for encorder error in PID mode

#define PID_1_KP    1.0     // Motor 1 PID proportional coefficient
#define PID_1_KI    0.1     // Motor 1 PID integral coefficient
#define PID_1_KD    0.0     // Motor 1 PID derivative coefficient
#define PID_1_POLE  10.0    // Motor 1 PID dirty derivative pole
#define PID_1_SAT   100.0   // Motor 1 PID integral saturation

#define PID_2_KP    1.0     // Motor 2 PID proportional coefficient
#define PID_2_KI    0.1     // Motor 2 PID integral coefficient
#define PID_2_KD    0.0     // Motor 2 PID derivative coefficient
#define PID_2_POLE  10.0    // Motor 2 PID dirty derivative pole
#define PID_2_SAT   100.0   // Motor 2 PID integral saturation

#define PID_3_KP    1.0     // Motor 3 PID proportional coefficient
#define PID_3_KI    0.1     // Motor 3 PID integral coefficient
#define PID_3_KD    0.0     // Motor 3 PID derivative coefficient
#define PID_3_POLE  10.0    // Motor 3 PID dirty derivative pole
#define PID_3_SAT   100.0   // Motor 3 PID integral saturation

#define PID_4_KP    1.0     // Motor 4 PID proportional coefficient
#define PID_4_KI    0.1     // Motor 4 PID integral coefficient
#define PID_4_KD    0.0     // Motor 4 PID derivative coefficient
#define PID_4_POLE  10.0    // Motor 4 PID dirty derivative pole
#define PID_4_SAT   100.0   // Motor 4 PID integral saturation

#define PID_5_KP    1.0     // Motor 5 PID proportional coefficient
#define PID_5_KI    0.1     // Motor 5 PID integral coefficient
#define PID_5_KD    0.0     // Motor 5 PID derivative coefficient
#define PID_5_POLE  10.0    // Motor 5 PID dirty derivative pole
#define PID_5_SAT   100.0   // Motor 5 PID integral saturation

#define PID_6_KP    1.0     // Motor 6 PID proportional coefficient
#define PID_6_KI    0.1     // Motor 6 PID integral coefficient
#define PID_6_KD    0.0     // Motor 6 PID derivative coefficient
#define PID_6_POLE  10.0    // Motor 6 PID dirty derivative pole
#define PID_6_SAT   100.0   // Motor 6 PID integral saturation


// ============================================================
// Components & Variables
// ============================================================

PinControl mot1_ina = PinControl(MOTOR_1_INA);
PinControl mot1_inb = PinControl(MOTOR_1_INB);
PinControl mot1_pwm = PinControl(MOTOR_1_PWM);
PinMeasure mot1_cha = PinMeasure(MOTOR_1_CHA, true);
PinMeasure mot1_chb = PinMeasure(MOTOR_1_CHB, true);
PinMeasure mot1_end = PinMeasure(MOTOR_1_END);

PinControl mot2_ina = PinControl(MOTOR_2_INA);
PinControl mot2_inb = PinControl(MOTOR_2_INB);
PinControl mot2_pwm = PinControl(MOTOR_2_PWM);
PinMeasure mot2_cha = PinMeasure(MOTOR_2_CHA, true);
PinMeasure mot2_chb = PinMeasure(MOTOR_2_CHB, true);
PinMeasure mot2_end = PinMeasure(MOTOR_2_END);

PinControl mot3_ina = PinControl(MOTOR_3_INA);
PinControl mot3_inb = PinControl(MOTOR_3_INB);
PinControl mot3_pwm = PinControl(MOTOR_3_PWM);
PinMeasure mot3_cha = PinMeasure(MOTOR_3_CHA, true);
PinMeasure mot3_chb = PinMeasure(MOTOR_3_CHB, true);
PinMeasure mot3_end = PinMeasure(MOTOR_3_END);

PinControl mot4_ina = PinControl(MOTOR_4_INA);
PinControl mot4_inb = PinControl(MOTOR_4_INB);
PinControl mot4_pwm = PinControl(MOTOR_4_PWM);
PinMeasure mot4_cha = PinMeasure(MOTOR_4_CHA, true);
PinMeasure mot4_chb = PinMeasure(MOTOR_4_CHB, true);
PinMeasure mot4_end = PinMeasure(MOTOR_4_END);

PinControl mot5_ina = PinControl(MOTOR_5_INA);
PinControl mot5_inb = PinControl(MOTOR_5_INB);
PinControl mot5_pwm = PinControl(MOTOR_5_PWM);
PinMeasure mot5_cha = PinMeasure(MOTOR_5_CHA, true);
PinMeasure mot5_chb = PinMeasure(MOTOR_5_CHB, true);
PinMeasure mot5_end = PinMeasure(MOTOR_5_END);

PinControl mot6_ina = PinControl(MOTOR_6_INA);
PinControl mot6_inb = PinControl(MOTOR_6_INB);
PinControl mot6_pwm = PinControl(MOTOR_6_PWM);
PinMeasure mot6_cha = PinMeasure(MOTOR_6_CHA, true);
PinMeasure mot6_chb = PinMeasure(MOTOR_6_CHB, true);
PinMeasure mot6_end = PinMeasure(MOTOR_6_END);

Motor motor1 = Motor(mot1_ina, mot1_inb, mot1_pwm, mot1_cha, mot1_chb, mot1_end);
Motor motor2 = Motor(mot2_ina, mot2_inb, mot2_pwm, mot2_cha, mot2_chb, mot2_end);
Motor motor3 = Motor(mot3_ina, mot3_inb, mot3_pwm, mot3_cha, mot3_chb, mot3_end);
Motor motor4 = Motor(mot4_ina, mot4_inb, mot4_pwm, mot4_cha, mot4_chb, mot4_end);
Motor motor5 = Motor(mot5_ina, mot5_inb, mot5_pwm, mot5_cha, mot5_chb, mot5_end);
Motor motor6 = Motor(mot6_ina, mot6_inb, mot6_pwm, mot6_cha, mot6_chb, mot6_end);

PinControl enable = PinControl(MOTORS_EN);
PinControl toggle = PinControl(PIN_TOGGLE);
Robot robot = Robot(enable, toggle, TS, 6);


// ============================================================
// setup
// ============================================================

void setup()
{
  toggle.set(true);

  PWMfreq::set(PWMfreq::MegaTimer3::FREQ_3921_16);
  PWMfreq::set(PWMfreq::MegaTimer4::FREQ_3921_16);

  Serial.begin(BAUDRATE);
  Serial.flush();

  robot.setMotor(0, &motor1);
  robot.setMotor(1, &motor2);
  robot.setMotor(2, &motor3);
  robot.setMotor(3, &motor4);
  robot.setMotor(4, &motor5);
  robot.setMotor(5, &motor6);

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

  toggle.set(false);
}


// ============================================================
// Loop
// ============================================================

void loop()
{
  robot.cycle(millis());
}
