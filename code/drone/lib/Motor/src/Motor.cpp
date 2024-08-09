#include "Motor.h"

#define PIN_FRONT_RIGHT 27 // OK
#define PIN_REAR_RIGHT 26 // OK
#define PIN_REAR_LEFT 25 // 
#define PIN_FRONT_LEFT 32 // 32

Motor::Motor() {

};

void Motor::init(uint8_t MIN, uint8_t MAX) {
  this->min_throttle = MIN;
  this->max_throttle = MAX;
  ledcSetup(0, 200, 8);
  ledcAttachPin(PIN_FRONT_RIGHT, 0);
  ledcWrite(0, min_throttle);
  ledcSetup(1, 200, 8);
  ledcAttachPin(PIN_REAR_RIGHT, 1);
  ledcWrite(1, min_throttle);
  ledcSetup(2, 200, 8);
  ledcAttachPin(PIN_REAR_LEFT, 2);
  ledcWrite(2, min_throttle);
  ledcSetup(3, 200, 8);
  ledcAttachPin(PIN_FRONT_LEFT, 3);
  ledcWrite(3, min_throttle);
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

void Motor::frontRight(uint8_t pwm) {
  ledcWrite(0, pwm < min_throttle ? min_throttle : pwm );
}

void Motor::rearRight(uint8_t pwm) {
  ledcWrite(2, pwm < min_throttle ? min_throttle : pwm );
}

void Motor::rearLeft(uint8_t pwm) {
  ledcWrite(1, pwm < min_throttle ? min_throttle : pwm );
}

void Motor::frontLeft(uint8_t pwm) {
  ledcWrite(3, pwm < min_throttle ? min_throttle : pwm );
}

void Motor::rlSetOffset(uint8_t pwm) {
  this->rl_offset = pwm;
}

void Motor::rrSetOffset(uint8_t pwm) {
  this->rr_offset = pwm;
}

void Motor::lfSetOffset(uint8_t pwm) {
  this->lf_offset = pwm;
}

void Motor::lrSetOffset(uint8_t pwm) {
  this->lr_offset = pwm;
}

void Motor::arm() {
  for(int i = 0; i < max_throttle; i++) {
    ledcWrite(0, min_throttle+i);
    ledcWrite(1, min_throttle+i);
    ledcWrite(2, min_throttle+i);
    ledcWrite(3, min_throttle+i);
    delay(3);
  }
  for(int x = max_throttle; x > 1; x--) {
    ledcWrite(0, min_throttle+x);
    ledcWrite(1, min_throttle+x);
    ledcWrite(2, min_throttle+x);
    ledcWrite(3, min_throttle+x);
    delay(3);
  }
}

void Motor::test() {
  delay(1000);
  frontLeft(min_throttle+10);
  delay(1000);
  frontLeft(min_throttle);
  rearRight(min_throttle+10);
  delay(1000);
  rearRight(min_throttle);
  rearLeft(min_throttle+10);
  delay(1000);
  rearLeft(min_throttle);
  frontRight(min_throttle+10);
  delay(1000);
  frontRight(min_throttle);
  delay(1000);
}

