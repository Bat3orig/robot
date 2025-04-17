
// #define PASS_ROBOT
#define SHOT_ROBOT

#ifdef PASS_ROBOT
  #define PULL_PWM 100 
  #define PUSH_PWM -100
#else
  #define PULL_PWM 50
  #define PUSH_PWM -50
#endif

#define PASS_PWM 225  
#define PASS_DELAY 3000

#define SHOT_PWM 220
#define SHOT_DELAY 3000

#define LONG_SHOT_PWM 250
#define LONG_SHOT_DELAY 3000

#define DRIBBLE_PWM 250
#define DRIBBLE_DELAY 3000

#define ACCEPT_PWM -100

// суурийн эргэлтийн pwm
#define TURN_PWM 20
#define TURN_FAST_PWM 30

#define ENCODER_PULSE_PER_ROTATION 360
#define HAND_MAX_LOCATION 300


