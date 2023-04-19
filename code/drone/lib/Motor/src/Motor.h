#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
  public:
    Motor();
    void init();
    void setPwn(uint8_t);
    void front(uint8_t);
    void left(uint8_t);
    void right(uint8_t);
    void back(uint8_t);
    void rl(uint8_t);
    void rr(uint8_t);
    void lr(uint8_t);
    void lf(uint8_t);
    void test();
};

#endif