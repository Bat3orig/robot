#include "motor.h"
#include "math.h"

// Motor* Motor::_motorInstance = nullptr;

Motor::Motor(uint8_t pwm, uint8_t dirA, uint8_t dirB) {
  _pwm = pwm;
  _dirA = dirA;
  _dirB = dirB;
}

void Motor::init() {
  _lastTime = millis(); 

  pinMode(_pwm, OUTPUT);
  pinMode(_dirA, OUTPUT);
  pinMode(_dirB, OUTPUT);
}

void Motor::setSpeed(float targetSpeed) {
  int pwmValue = constrain(targetSpeed, -MAX_SPEED, MAX_SPEED);

  if(pwmValue > 0) {
    setDirection(true);
  }
  else if(pwmValue < 0) {
    setDirection(false);
  }
  else {
    stop();
    return;
  } 

  analogWrite(_pwm, abs(pwmValue)); 
}

void Motor::stop() {
  analogWrite(_pwm, 250); 
  digitalWrite(_dirA, HIGH);
  digitalWrite(_dirB, HIGH);
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