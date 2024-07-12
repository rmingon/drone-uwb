#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "config.h"

class Motor {
  private:
    uint8_t rl_offset;
    uint8_t rr_offset;
    uint8_t lr_offset;
    uint8_t lf_offset;
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
    void rlSetOffset(uint8_t);
    void rrSetOffset(uint8_t);
    void lrSetOffset(uint8_t);
    void lfSetOffset(uint8_t);
    void test();
};

#endif