/*
 * Төмөр замын сургууль
 * Робокон 2015
 * Моторын удирдлагын library
*/

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

#define MAX_SPEED 100

class Motor {

  public:
    Motor(uint8_t pwm, uint8_t dirA, uint8_t dirB);
    void init();
    void setSpeed(float targetSpeed);
    void stop();
    void setDirection(bool direction);  // true = clockwise, false = counter-clockwise
    
  
  private:
    uint8_t _pwm;
    uint8_t _dirA;
    uint8_t _dirB;
    unsigned long _lastTime;
};

#endif"