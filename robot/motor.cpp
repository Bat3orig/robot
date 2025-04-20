#include "motor.h"
#include "math.h"

// Motor* Motor::_motorInstance = nullptr;

Motor::Motor(uint8_t type, uint8_t dirA, uint8_t dirB, uint8_t pwm) {
  _type = type;
  _pwm = pwm;
  _dirA = dirA;
  _dirB = dirB;
}

void Motor::init(uint8_t max_speed) {
  this->max_speed = max_speed;

  if(_type == POLOLU) {
    pinMode(_pwm, OUTPUT);
    pinMode(_dirA, OUTPUT);
    pinMode(_dirB, OUTPUT);
  }
  else if(_type == LBT2) {
    pinMode(_dirA, OUTPUT);
    pinMode(_dirB, OUTPUT);
  }
}

void Motor::setSpeed(float targetSpeed) {
  int pwmValue = constrain(targetSpeed, -max_speed, max_speed);

  if(_type == POLOLU) {
    if(pwmValue > 0) {
      setDirection(true);
      analogWrite(_pwm, pwmValue);
    }
    else if(pwmValue < 0) {
      setDirection(false);
      analogWrite(_pwm, abs(pwmValue));
    }
    else {
      stop(true); // with brake
    }
  }
  else if(_type == LBT2) {
    if(pwmValue > 0) {
      analogWrite(_dirA, pwmValue);
      digitalWrite(_dirB, 0);
    }
    else if(pwmValue < 0) {
      digitalWrite(_dirA, 0);
      analogWrite(_dirB, abs(pwmValue));
    }
    else {
      stop();
    } 
  }
}

void Motor::stop(bool brake) {
  if(_type == POLOLU)
    digitalWrite(_pwm, HIGH); 
  digitalWrite(_dirA, brake);
  digitalWrite(_dirB, brake);
}

void Motor::setDirection(bool direction) {
  if(direction) {
    digitalWrite(_dirA, LOW);
    digitalWrite(_dirB, HIGH);
  }
  else {
    digitalWrite(_dirA, HIGH);
    digitalWrite(_dirB, LOW);
  }
}