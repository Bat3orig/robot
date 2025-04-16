
// #define PASS_ROBOT
#define SHOT_ROBOT

//GPIO
#define LPMS_SERIAL Serial2
#define PIN_PS2_CLK 22  // CLK
#define PIN_PS2_CMD 25  // MOSI
#define PIN_PS2_CS 24   // Chip select
#define PIN_PS2_DAT 23  // MISO

#define PIN_EN_A 18
#define PIN_EN_B 19
#define PIN_LIMIT_SWITCH 47
#define PIN_SHOT1_PWM 6    
#define PIN_ACCEPT1_PWM 7
#define PIN_SHOT2_PWM 10
#define PIN_ACCEPT2_PWM 11
#define PIN_PULL_PWM 9
#define PIN_PUSH_PWM 8
#define PIN_PUSH_BALL 49
#define PIN_STAND 53
#define PIN_STRETCH 51

#define NO_ROTATE 0
#define ROTATE_LEFT_SLOW 1
#define ROTATE_RIGHT_SLOW 2
#define ROTATE_LEFT_FAST 3
#define ROTATE_RIGHT_FAST 4

#ifdef PASS_ROBOT
#define PULL_PWM 100 
#define PUSH_PWM -100
#else
#define PULL_PWM 50
#define PUSH_PWM -50
#endif

#define PASS_PWM 200        
#define SHOT_PWM 220
#define LONG_SHOT_PWM 250
#define DRIBBLE_PWM 250
#define ACCEPT_PWM 100


#define ENCODER_PULSE_PER_ROTATION 360
#define HAND_MAX_LOCATION 800

// #define INITIAL_LOCATION 0
// #define SHOT_LOCATION 1
// #define LONG_SHOT_LOCATION 2
// #define PASS_LOCATION 3
// #define ZALAH_LOCATION 4 


