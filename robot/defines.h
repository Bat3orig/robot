
#define PASS_ROBOT
// #define SHOT_ROBOT

#ifdef SHOT_ROBOT
  #define PULL_PWM 200 
  #define PUSH_PWM -200
#else
  #define PULL_PWM 100
  #define PUSH_PWM -100
#endif

#define PASS_PWM 225  
#define PASS_DELAY 2300

#define SHOT_PWM 220
#define SHOT_DELAY 2300

#define LONG_SHOT_PWM 250
#define LONG_SHOT_DELAY 2300

#define DRIBBLE_PWM 150
#define DRIBBLE_DELAY 2300

#define ACCEPT_PWM -100

// суурийн эргэлтийн pwm
#define TURN_PWM 20
#define TURN_FAST_PWM 30

#define ENCODER_PULSE_PER_ROTATION 360
#define HAND_MAX_LOCATION 300


