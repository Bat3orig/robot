#include "defines.h"
#include "lpms.h" 
#include "motor.h"
#include "PS2X_lib.h"

#define LPMS_SERIAL Serial1

// PS2 GPIO
#define PIN_PS2_CLK 22  // CLK
#define PIN_PS2_CMD 25  // MOSI
#define PIN_PS2_CS 24   // Chip select
#define PIN_PS2_DAT 23  // MISO

// ENCODER GPIO
#define PIN_EN_A 20
#define PIN_EN_B 21

// SWITCH AND BUTTON PINS
#define PIN_LIMIT_SWITCH 47

// RELAY PINS OF PINEMATIC
uint8_t RELAY_PINS[3] = {49, 51, 53};   // PUSH BALL, STRETCH, STAND
#define BALL 0
#define STAND 1
#define STRETCH 2

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
#ifdef SHOT_ROBOT
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
#else
  // PASS ROBOT
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

LPMS lpms(LPMS_SERIAL);
PS2X ps2;

typedef struct {
  float speed;
  float rotate;
  float direction;
} SUURI_PARAMS;

SUURI_PARAMS current, target;

int encoder = 0;
bool lpms_running = false;
float euler = 0;  // lpms - ийн өнцөг
float angle = 0;  // robot - ийн эргүүлэх өнцөг
bool stand = false;
bool stretch = false;

void setup() {
  Serial.begin(115200); // simulator
  lpms.begin(115200);
  motor1.init(100);
  motor2.init(100);
  motor3.init(100);
  motor4.init(100);
  motor_hand.init(100);
  motor_shot_up.init(250);
  motor_shot_down.init(250);
  pinMode(PIN_EN_A, INPUT_PULLUP);
  pinMode(PIN_EN_B, INPUT_PULLUP);
  pinMode(PIN_LIMIT_SWITCH, INPUT);
  for(uint8_t i = 0; i < 3; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], HIGH);
  }

  attachInterrupt(digitalPinToInterrupt(PIN_EN_A), encoderISR, RISING);

  delay(5000); // wait for LPMS power on

  // ps2.begin(PIN_PS2_CLK, PIN_PS2_CMD, PIN_PS2_CS, PIN_PS2_DAT);  // clk, cmd, att, dat
  ps2.config_gamepad(PIN_PS2_CLK, PIN_PS2_CMD, PIN_PS2_CS, PIN_PS2_DAT);  // clk, cmd, att, dat

  if(lpms.setMode(COMMAND_MODE)) {
    Serial.println("Command mode success");
    delay(2000);
    if(lpms.setOffset()) {
      Serial.println("Setoffset success");
      lpms_running = true;
    }
  }
  // init hand location
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
  /***************** LPMS - ийн өнцгийг унших ***************************/
  if(lpms_running) {
    if (lpms.requestAngle()) {
      euler = lpms.euler[Z];
    }
  }

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
    motor_shot_up.stop();
    motor_shot_down.stop();
  }
  else {          // go
    move();
  }

  /***************** Pinematic  ******************/
  if(ps2.ButtonPressed(PSB_SQUARE)) {
    stand = !stand;
    digitalWrite(RELAY_PINS[STAND], stand);
  }
  if(ps2.ButtonPressed(PSB_CIRCLE)) {
    stretch = !stretch;
    digitalWrite(RELAY_PINS[STRETCH], stretch);
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
  if(ps2.ButtonPressed(PSB_L1)) {
    motor_shot_up.stop();
    motor_shot_down.setSpeed(DRIBBLE_PWM);
    shot(DRIBBLE_DELAY);
    delay(1000);
    motor_shot_up.setSpeed(ACCEPT_PWM);
    motor_shot_down.setSpeed(ACCEPT_PWM);
  }
  else if(ps2.ButtonPressed(PSB_L2)) {
    motor_shot_up.setSpeed(PASS_PWM);
    motor_shot_down.setSpeed(PASS_PWM);
    shot(PASS_DELAY);
  }
  else if(ps2.ButtonPressed(PSB_R1)) {
    motor_shot_up.setSpeed(SHOT_PWM);
    motor_shot_down.setSpeed(SHOT_PWM);
    shot(SHOT_DELAY);

  }
  else if(ps2.ButtonPressed(PSB_R2)) {
    motor_shot_up.setSpeed(LONG_SHOT_PWM);
    motor_shot_down.setSpeed(LONG_SHOT_PWM);
    shot(LONG_SHOT_DELAY);
  }
  else if(ps2.ButtonPressed(PSB_START)) {
    motor_shot_up.setSpeed(ACCEPT_PWM);
    motor_shot_down.setSpeed(ACCEPT_PWM);
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
    if(lpms_running) {
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

void encoderISR() {
  if(digitalRead(PIN_EN_B) == LOW)
    encoder++;
  else
    encoder--;
}

void shot(int duration) {
  delay(duration); // Мотор бүрэн эргэлтээ авах хүртэл хүлээх
  digitalWrite(RELAY_PINS[BALL], LOW);
  delay(500);
  digitalWrite(RELAY_PINS[BALL], HIGH);
  motor_shot_up.stop();
  motor_shot_down.stop();

}

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

void simulate() {
  float direction = 0, targetSpeed = 0, angle_speed = 0;
  int hand_speed = 0;
  while(1) {
    if(lpms_running) {
      if (lpms.requestAngle()) {
        euler = lpms.euler[Z];
      }
    }

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
      // Шидэлт
      else if(inputString.startsWith("dribble")) {
        int speed = getValue(inputString, "pwm").toInt();
        unsigned long duration = getValue(inputString, "time").toInt();
        motor_shot_up.stop();
        motor_shot_down.setSpeed(speed);
        shot(duration);
        delay(500);
        motor_shot_up.setSpeed(ACCEPT_PWM);
        motor_shot_up.setSpeed(ACCEPT_PWM);
      }
      else if(inputString.startsWith("shot")) {         // shot and pass
        int speed = getValue(inputString, "pwm").toInt();
        unsigned long duration = getValue(inputString, "time").toInt();
        motor_shot_up.setSpeed(speed);
        motor_shot_down.setSpeed(speed);
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