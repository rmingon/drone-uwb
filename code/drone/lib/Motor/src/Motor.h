#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
  private:
    uint8_t rl_offset;
    uint8_t rr_offset;
    uint8_t lr_offset;
    uint8_t lf_offset;
    uint8_t min_throttle;
    uint8_t max_throttle;
  public:
    Motor();
    void init(uint8_t, uint8_t);
    void arm();
    void setPwn(uint8_t);
    void front(uint8_t);
    void left(uint8_t);
    void right(uint8_t);
    void back(uint8_t);
    void frontRight(uint8_t);
    void rearRight(uint8_t);
    void frontLeft(uint8_t);
    void rearLeft(uint8_t);
    void rlSetOffset(uint8_t);
    void rrSetOffset(uint8_t);
    void lrSetOffset(uint8_t);
    void lfSetOffset(uint8_t);
    void test();
};

#endif