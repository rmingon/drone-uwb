#include "Motor.h"

#define PIN_RF 25
#define PIN_RR 26
#define PIN_LR 27
#define PIN_LF 32

Motor::Motor() {

};

void Motor::init() {
  ledcSetup(0, 1000, 8);
  ledcAttachPin(PIN_RF, 0);
  ledcSetup(1, 1000, 8);
  ledcAttachPin(PIN_RR, 1);
  ledcSetup(2, 1000, 8);
  ledcAttachPin(PIN_LR, 2);
  ledcSetup(3, 1000, 8);
  ledcAttachPin(PIN_LF, 3);
  setPwn(0);
}

void Motor::setPwn(uint8_t pwm) {
  ledcWrite(0, pwm);
  ledcWrite(1, pwm);
  ledcWrite(2, pwm);
  ledcWrite(3, pwm);
};

void Motor::front(uint8_t pwm) {
  ledcWrite(0, pwm);
  ledcWrite(1, pwm);
}

void Motor::left(uint8_t pwm) {
  ledcWrite(1, pwm);
  ledcWrite(3, pwm);
}

void Motor::right(uint8_t pwm) {
  ledcWrite(2, pwm);
  ledcWrite(0, pwm);
}

void Motor::back(uint8_t pwm) {
  ledcWrite(3, pwm);
  ledcWrite(2, pwm);
}

void Motor::rl(uint8_t pwm) {
  ledcWrite(0, pwm);
}

void Motor::rr(uint8_t pwm) {
  ledcWrite(2, pwm);
}

void Motor::lf(uint8_t pwm) {
  ledcWrite(1, pwm);
}

void Motor::lr(uint8_t pwm) {
  ledcWrite(3, pwm);
}

