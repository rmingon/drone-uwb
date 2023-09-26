#ifndef _MOTOR_H_
#define _MOTOR_H_

typedef struct {
    int l;
    int r;
    int b;
    int t;
} MotorsSpeed;

void motorInit();

void motorControl(void *pvParameters);

#endif
