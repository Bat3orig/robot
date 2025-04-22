#include <Wire.h>
#include "defines.h"
#include "motor.h"
#include "PS2X_lib.h"

#ifdef SHOT_ROBOT
  // shot robot has LPMS-ME1 9 axis sensor
  #include "lpms.h" 
  #define PIN_EN_A 20
  #define PIN_EN_B 21
  Motor motor1(POLOLU, 48, 46, 4);  // type, A, B and pwm. pwm is optional
  Motor motor2(POLOLU, 50, 52, 5);  
  Motor motor3(POLOLU, 40, 38, 2);
  Motor motor4(POLOLU, 44, 42, 3);
  float rotate_scale = 3; // angle * rotate_scle => pwm
  float accel = 1;  // 100 => 1 + 1 + 1 + = 100. 0 => 100 -1 - 1- 1 0
  float scale_m1 = 1; //60 = 60
  float scale_m2 = 1; // 60 = 60
  float scale_m3 = 1; // 60 = 52
  float scale_m4 = 1; // 60 = 72
  LPMS lpms(Serial1);
#else
  // PASS ROBOT has MPU9250 9 axis sensor
  Motor motor1(POLOLU, 46, 48, 4);  // pwm, A, B
  Motor motor2(POLOLU, 52, 50, 5);  
  Motor motor3(POLOLU, 38, 40, 2);
  Motor motor4(POLOLU, 44, 42, 3);
  float rotate_scale = 3; // angle * rotate_scle => pwm
  float accel = 1;  // 100 => 1 + 1 + 1 + = 100. 0 => 100 -1 - 1- 1 0
  float scale_m1 = 1; //60 = 60
  float scale_m2 = 1; // 60 = 60
  float scale_m3 = 0.85; // 60 = 52
  float scale_m4 = 1.25; // 60 = 72
#endif


// PS2 GPIO
#define PIN_PS2_CLK 22  // CLK
#define PIN_PS2_CMD 25  // MOSI
#define PIN_PS2_CS 24   // Chip select
#define PIN_PS2_DAT 23  // MISO
#define PIN_LIMIT_SWITCH 47
#define SHOT_RELAY 51
#define STRETCH_RELAY 53

#define IDLE 0
#define PASS 1
#define DRIBBLE 2
#define SHOT 3
#define LONG_SHOT 4
#define ACCEPT 5

Motor motor_shot_up(LBT2, 10, 11);
Motor motor_shot_down(LBT2, 6, 7);
Motor motor_hand(LBT2, 8, 9);

/* Моторын байрлал
     3     4
      /   \
      \   /
     2     1     
  <-- positive rotation  A = high B = low
  --> negative rotation
*/


PS2X ps2;

typedef struct {
  float speed;
  float rotate;
  float direction;
} SUURI_PARAMS;

SUURI_PARAMS current, target;

int encoder = 0;
bool gyro_running = false;
float euler = 0;  // lpms - ийн өнцөг
float angle = 0;  // robot - ийн эргүүлэх өнцөг
uint8_t shot_state = IDLE;
uint8_t next_shot_state = IDLE;
uint32_t shot_millis = 0;
bool stretch = false;

void setup() {
  Serial.begin(115200); // simulator
  Wire.begin();
  delay(2000);
  #ifdef SHOT_ROBOT // Init LPMS-ME1
    pinMode(PIN_EN_A, INPUT_PULLUP);
    pinMode(PIN_EN_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_EN_A), encoderISR, RISING);
    motor1.init(100);
    motor2.init(100);
    motor3.init(100);
    motor4.init(100);
    lpms.begin(115200);
    if(lpms.setMode(COMMAND_MODE)) {
      Serial.println("Command mode success");
      delay(2000);
      if(lpms.setOffset()) {
        Serial.println("Setoffset success");
        gyro_running = true;
      }
    }
  #else   // PASS_ROBOT. Init MPU9250
    Serial1.begin(9600);
    motor1.init(100);
    motor2.init(100);
    motor3.init(100);
    motor4.init(100);
    gyro_running = true;
  #endif
  motor_hand.init(200);
  motor_shot_up.init(250);
  motor_shot_down.init(250);
  pinMode(PIN_LIMIT_SWITCH, INPUT);
  pinMode(STRETCH_RELAY, OUTPUT);
  pinMode(SHOT_RELAY, OUTPUT);
  digitalWrite(STRETCH_RELAY, HIGH);
  digitalWrite(SHOT_RELAY, HIGH);

  // ps2.begin(PIN_PS2_CLK, PIN_PS2_CMD, PIN_PS2_CS, PIN_PS2_DAT);  // clk, cmd, att, dat
  ps2.config_gamepad(PIN_PS2_CLK, PIN_PS2_CMD, PIN_PS2_CS, PIN_PS2_DAT);  // clk, cmd, att, dat
  
  // unsigned long ms = millis();

  motor_hand.setSpeed(PULL_PWM);
  while(isLimitSwitchPressed());
  motor_hand.stop();
  delay(1000);
  motor_hand.setSpeed(PUSH_PWM);
  while(!isLimitSwitchPressed());
  motor_hand.stop();
  encoder = 0;

  // uncomment the following the lines for simulator of windows software.
  // Serial.println("log->Ready");
  // if(Serial) {
  //   simulate();
  // }
}

void loop() {
  /***************** Euler Z өнцгийг унших ***************************/
  #ifdef SHOT_ROBOT
    if(gyro_running) {
      if (lpms.requestAngle()) {
        euler = lpms.euler[Z];
      }
    }
  #else
    if(Serial1.available()) {
      String inputString = Serial1.readStringUntil('\n');
      inputString.trim();
      euler = inputString.toFloat();
    }
  #endif

  /***************** Joystick уншиж, боловсруулах **********/
  ps2.read_gamepad();
  process_joystick(); // calculate speed, direction and rotate speed
  
  /***************** Суурь явалт ******************/
  if(ps2.Button(PSB_CROSS)) { // brake motors if X button pressed
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
    angle = euler;
    current.speed = 0;
    target.speed = 0;
    current.rotate = 0;
    target.rotate = 0;
    motor_shot_up.stop();
    motor_shot_down.stop();
  }
  else {          // go
    move();
  }

  /***************** Pinematic  ******************/
  if(ps2.Button(PSB_TRIANGLE)) 
    digitalWrite(SHOT_RELAY, LOW);
  else
    digitalWrite(SHOT_RELAY, HIGH);
  if(ps2.ButtonPressed(PSB_SQUARE)) {
    digitalWrite(STRETCH_RELAY, stretch);
    stretch = !stretch;
  }


  // /***************** Шидэлтийн өнцөг тохируулалт ******************/
  if(ps2.Button(PSB_PAD_UP)) {
    if(isLimitSwitchPressed())
      motor_hand.stop();
    else
      motor_hand.setSpeed(PUSH_PWM);
  }
  else if(ps2.Button(PSB_PAD_DOWN)){
    if(encoder >= HAND_MAX_LOCATION)
      motor_hand.stop();
    else
      motor_hand.setSpeed(PULL_PWM);
  }
  else {
    motor_hand.stop();
  }

  // /***************** Шидэлт ******************/
  if(ps2.ButtonPressed(PSB_L1)) {           // DRIBLE ROTATION
    shot_millis = millis();
    next_shot_state = DRIBBLE;
  }
  else if(ps2.ButtonPressed(PSB_L2)) {      // PASS ROTATION
    if(shot_state == DRIBBLE || shot_state == ACCEPT) {
      shot_millis = millis();
    }
    next_shot_state = PASS;
  }
  else if(ps2.ButtonPressed(PSB_R1)) {      // SHOT ROTATION
    if(shot_state == DRIBBLE || shot_state == ACCEPT) {
      shot_millis = millis();
    }
    next_shot_state = SHOT;
    
  }
  else if(ps2.ButtonPressed(PSB_R2)) {      // 
    if(shot_state == DRIBBLE || shot_state == ACCEPT) {
      shot_millis = millis();
    }
    next_shot_state = LONG_SHOT;
    
  }
  else if(ps2.ButtonPressed(PSB_START)) {
    shot_millis = millis();
    next_shot_state = ACCEPT;
  }
  else if(ps2.ButtonPressed(PSB_CIRCLE)) {
    next_shot_state = IDLE;
  }


  if(shot_state != next_shot_state) {
    if(millis() - shot_millis > 2000)
      shot_state = next_shot_state;
    else {
      motor_shot_up.stop();
      motor_shot_down.stop();
    }
  }
  else {
    if(shot_state == IDLE) {
      motor_shot_up.stop();
      motor_shot_down.stop();
    }
    else if(shot_state == DRIBBLE) {
      motor_shot_up.setSpeed(-DRIBBLE_PWM/3);
      motor_shot_down.setSpeed(DRIBBLE_PWM);
    }
    else if(shot_state == PASS) {
      motor_shot_up.setSpeed(PASS_PWM);
      motor_shot_down.setSpeed(PASS_PWM);
    }
    else if(shot_state == SHOT) {
      motor_shot_up.setSpeed(SHOT_PWM);
      motor_shot_down.setSpeed(SHOT_PWM);
    }
    else if(shot_state == LONG_SHOT) {
      motor_shot_up.setSpeed(LONG_SHOT_PWM);
      motor_shot_down.setSpeed(LONG_SHOT_PWM);
    }
    else if(shot_state == ACCEPT) {
      motor_shot_up.setSpeed(ACCEPT_PWM);
      motor_shot_down.setSpeed(ACCEPT_PWM);
    }
  }
  
}


void move() {
  if(current.speed != target.speed) {
    if(current.speed < target.speed)
      current.speed += accel;
    else
      current.speed -= (accel * 2);
  }

  if(current.rotate != target.rotate) {
    if(current.rotate < target.rotate)
      current.rotate += 0.5;
    else 
      current.rotate -= 0.5;
  }

  float Vx = current.speed * cos(target.direction * DEG_TO_RAD);
  float Vy = current.speed * sin(target.direction * DEG_TO_RAD);

  float theta = 0;

  if(target.rotate == 0) {
    if(gyro_running) {
      theta = angle - euler;
      if (theta > 180) theta -= 360;
      if (theta < -180) theta += 360;
    }
  }
  else {
    angle = euler;
  }

  float w1 = -Vx - Vy - current.rotate - theta * rotate_scale *2;
  float w2 = -Vx + Vy - current.rotate - theta * rotate_scale * 2;
  float w3 = Vx + Vy - current.rotate - theta * rotate_scale * 2;
  float w4 = Vx - Vy - current.rotate  - theta * rotate_scale * 2;

  motor1.setSpeed(w1 * scale_m1);
  motor2.setSpeed(w2 * scale_m2);
  motor3.setSpeed(w3 * scale_m3);
  motor4.setSpeed(w4 * scale_m4);

}

#ifdef SHOT_ROBOT 
void encoderISR() {
  if(digitalRead(PIN_EN_B) == LOW)
    encoder++;
  else
    encoder--;
}
#endif

bool isLimitSwitchPressed() {
  if(digitalRead(PIN_LIMIT_SWITCH) == HIGH) {
    unsigned long checkMillis = millis();
    while(digitalRead(PIN_LIMIT_SWITCH) == HIGH) {
      if(millis() - checkMillis > 50)
        return true;
    } 
  }
  return false;
}

void process_joystick() {
/* Joystick coordinate
          0
          |
          |
   0-------------255  (X)
          |
          |         Center = 127,127
         255 (Y)
  */
  // Баруун талын joystick
  int rx = ps2.Analog(PSS_RX);
  int ry = ps2.Analog(PSS_RY);
  if( (rx > 124 && rx < 130) && (ry > 124 && ry < 130)) {
    target.speed = 0;
  }
  if( (rx > 130 || rx < 124) || (ry > 130 || ry < 124)) {
    int x = map(rx, 0, 255, -100, 100);
    int y = map(ry, 0, 255, 100, -100);
    target.speed = sqrt(x * x + y * y);
    target.direction = atan2(y, x) * RAD_TO_DEG;
  }

  // Зүүн талын Joystick
  int lx = ps2.Analog(PSS_LX);
  int ly = ps2.Analog(PSS_LY);
  if((lx > 110 && lx < 140) && (ly > 110 && ly < 140)) {
    target.rotate = 0;
  }
  if((lx < 110 || lx > 140) || (ly > 140 || ly < 110)) {
    // joystick - ийн 0 - 255 утгыг нэмэх, хасах утга руу хөрвүүлэх
    int x = map(lx, 0, 255, -100, 100);
    int y = map(ly, 0, 255, 100, -100);
    if( x < -80)
      target.rotate = TURN_FAST_PWM;
    else if(x > 80)
      target.rotate = -TURN_FAST_PWM;
    else if(y < -80 )
      target.rotate = -TURN_PWM; 
    else if(y > 80) 
      target.rotate = TURN_PWM;
  }
}

// get value from string which incoming from simulator
String getValue(String data, String key) {
  int keyIndex = data.indexOf(key + "=");
  if (keyIndex == -1) return "";

  int start = keyIndex + key.length() + 1;
  int end = data.indexOf(',', start);
  if (end == -1) end = data.length();

  return data.substring(start, end);
}

void shot(int duration) {
  delay(duration); // Мотор бүрэн эргэлтээ авах хүртэл хүлээх
  digitalWrite(SHOT_RELAY, LOW);
  delay(500);
  digitalWrite(SHOT_RELAY, HIGH);
  motor_shot_up.stop();
  motor_shot_down.stop();

}

void simulate() {
  float direction = 0, targetSpeed = 0, angle_speed = 0;
  int hand_speed = 0;
  while(1) {
    #ifdef SHOT_ROBOT
      if(gyro_running) {
        if (lpms.requestAngle()) {
          euler = lpms.euler[Z];
        }
      }
    #else
      if(Serial1.available()) {
        String inputString = Serial1.readStringUntil('\n');
        inputString.trim();
        euler = inputString.toFloat();
      }
    #endif

    if(Serial.available()) {
      String inputString = Serial.readStringUntil('\n');
      // move,dir=10,speed=12,scale=0.1,accel=0.1\n
      // stop\n
      inputString.trim();
      // Суурь явалт
      if (inputString.startsWith("move")) {
          target.direction = getValue(inputString, "dir").toFloat();
          target.speed = getValue(inputString, "speed").toFloat();
          target.rotate = 0;
          Serial.print("log->Speed: ");
          Serial.print(targetSpeed);
          Serial.print("\tdir: ");
          Serial.println(direction);
      }
      else if(inputString.startsWith("turn")) {
        target.rotate = getValue(inputString, "pwm").toInt();
      }
      else if(inputString.startsWith("scale")) {
        scale_m1 = getValue(inputString, "m1").toFloat();
        scale_m2 = getValue(inputString, "m2").toFloat();
        scale_m3 = getValue(inputString, "m3").toFloat();
        scale_m4 = getValue(inputString, "m4").toFloat();
        accel = getValue(inputString, "accel").toFloat();
        rotate_scale = getValue(inputString, "scale").toFloat();

        Serial.print("log->accel: "); Serial.print(accel);
        Serial.print(", rotate_scale: "); Serial.print(rotate_scale);
        Serial.print(", m1: "); Serial.print(scale_m1);
        Serial.print(", m2: "); Serial.print(scale_m2);
        Serial.print(", m3: "); Serial.print(scale_m3);
        Serial.print(", m4: "); Serial.println(scale_m4);
      }
      else if(inputString.startsWith("stop")) {
        target.speed = 0;
        target.rotate = 0;
      }
      // Шидэх өнцөг
      else if(inputString.startsWith("hand")) {
        hand_speed = getValue(inputString, "pwm").toInt();
      }
      // Шидэлтg
      else if(inputString.startsWith("dribble")) {
        int speed1 = getValue(inputString, "pwm1").toInt();
        int speed2 = getValue(inputString, "pwm2").toInt();
        unsigned long duration = getValue(inputString, "time").toInt();
        motor_shot_up.setSpeed(speed1);
        motor_shot_down.setSpeed(speed2);
        shot(duration);
        // delay(500);
        // motor_shot_up.setSpeed(ACCEPT_PWM);
        // motor_shot_up.setSpeed(ACCEPT_PWM);
      }
      else if(inputString.startsWith("shot")) {         // shot and pass
        int speed1 = getValue(inputString, "pwm1").toInt();
        int speed2 = getValue(inputString, "pwm2").toInt();
        unsigned long duration = getValue(inputString, "time").toInt();
        motor_shot_up.setSpeed(speed1);
        motor_shot_down.setSpeed(speed2);
        shot(duration);
      }
      else if(inputString.startsWith("cancel")) {
        motor_shot_up.stop();
        motor_shot_down.stop();
      }
    }

    move();

     Serial.print("euler=");
     Serial.print(euler);
     Serial.print(",angle=");
     Serial.print(angle);
     Serial.print(",speed=");
     Serial.print(current.speed);
     Serial.print(",dir=");
     Serial.print(target.direction);
     Serial.print(",enc=");
     Serial.println(encoder);

    if(hand_speed == 0) 
      motor_hand.stop();
    else if(hand_speed < 0) {
      if(isLimitSwitchPressed()){
        hand_speed = 0;
        encoder = 0;
      }
      else
        motor_hand.setSpeed(hand_speed);
    }
    else if(hand_speed > 0) {
      if(encoder >= HAND_MAX_LOCATION)
        hand_speed = 0;
      else 
        motor_hand.setSpeed(hand_speed);
    }
  }
}
