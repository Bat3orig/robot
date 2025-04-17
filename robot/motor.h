/*
 * Төмөр замын сургууль
 * Робокон 2015
 * Моторын удирдлагын library
*/

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
  #define POLOLU 0
  #define LBT2 1
  public:
    Motor(uint8_t type, uint8_t dirA, uint8_t dirB, uint8_t pwm = 0);
    void init(uint8_t max_speed);
    void setSpeed(float targetSpeed);
    void stop(bool brake = false);
    void setDirection(bool direction);  // true = clockwise, false = counter-clockwise
    int max_speed;
  
  private:
    uint8_t _type;
    uint8_t _pwm;
    uint8_t _dirA;
    uint8_t _dirB;
};

#endif"