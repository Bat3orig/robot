#include "motor.h"
#include "lpms.h"
#include "PS2X_lib.h"

// roate zassan

// #define PASS_ROBOT
#define SHOT_ROBOT

//GPIO
#define LPMS_SERIAL Serial1
#define PS2_CLK 22  // CLK
#define PS2_CMD 25  // MOSI
#define PS2_CS 24   // Chip select
#define PS2_DAT 23  // MISO

#define EN_A 20
#define EN_B 21
#define LIMIT_SWITCH 49
#define SHOT1_PWM 6
#define ACCEPT1_PWM 7
#define SHOT2_PWM 12
#define ACCEPT2_PWM 13
#define PULL_PWM 8
#define PUSH_PWM 9
#define PUSH_BALL 53
#define RELAY_STAND 51
#define RELAY_STRETCH 49

#define NO_ROTATE 0
#define ROTATE_LEFT_SLOW 1
#define ROTATE_RIGHT_SLOW 2
#define ROTATE_LEFT_FAST 3
#define ROTATE_RIGHT_FAST 4


#define PASS_PWM 20
#define SHORT_SHOT_PWM 30
#define SHOT_PWM 40
#define LONG_SHOT_PWM 50
#define DRIBBLE_PWM 60


#define ENCODER_PULSE_PER_ROTATION 360

/* Моторын байрлал
     3     4
      /   \
      \   /
     2     1     
  <-- positive rotation  A = high B = low
  --> negative rotation
*/
Motor motor1(4, 46, 48);
Motor motor2(5, 52, 50);
Motor motor3(2, 38, 40);
Motor motor4(3, 44, 42);

// Motor motor1(4, 48, 46);  // pwm, A, B
// Motor motor2(5, 50, 52);  
// Motor motor3(2, 40, 38);
// Motor motor4(3, 44, 42);

LPMS lpms(LPMS_SERIAL);
PS2X ps2x;

#ifdef PASS_ROBOT
  // suuri motor params
float rotate_scale = 3; // angle * rotate_scle => pwm
float accel = 1;  // 100 => 1 + 1 + 1 + = 100. 0 => 100 -1 - 1- 1 0
float scale_m1 = 1; //60 = 60
float scale_m2 = 1; // 60 = 60
float scale_m3 = 0.85; // 60 = 52
float scale_m4 = 1.25; // 60 = 72
#endif

#ifdef SHOT_ROBOT
// suuri motor params
float rotate_scale = 3; // angle * rotate_scle => pwm
float accel = 1;  // 100 => 1 + 1 + 1 + = 100. 0 => 100 -1 - 1- 1 0
float scale_m1 = 1; //60 = 60
float scale_m2 = 1; // 60 = 60
float scale_m3 = 0.85; // 60 = 52
float scale_m4 = 1.25; // 60 = 72
#endif


typedef struct {
  float targetSpeed;
  float currentSpeed;
  float angle;
  float direction;
  uint8_t rotate;
  int32_t encoder;
  bool brake;
} MOTOR_PARAMS;

MOTOR_PARAMS suuri, shot, hand;

float euler = 0;      // lpms - ийн өнцөг
bool stand = false;
bool stretch = false;

void setup() {
  Serial.begin(115200); // simulator
  lpms.begin(115200);
  motor1.init();
  motor2.init();
  motor3.init();
  motor4.init();
  pinMode(EN_A, INPUT_PULLUP);
  pinMode(EN_B, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH, INPUT);
  pinMode(SHOT1_PWM, OUTPUT);
  pinMode(SHOT2_PWM, OUTPUT);
  pinMode(ACCEPT1_PWM, OUTPUT);
  pinMode(ACCEPT2_PWM, OUTPUT);
  pinMode(PULL_PWM, OUTPUT);
  pinMode(PUSH_PWM, OUTPUT);
  pinMode(PUSH_BALL, OUTPUT);
  pinMode(RELAY_STAND, OUTPUT);
  pinMode(RELAY_STRETCH, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(EN_A), encoderISR, RISING);

  digitalWrite(PULL_PWM, 0);
  digitalWrite(PUSH_PWM, 0);
  digitalWrite(SHOT1_PWM, 0);
  digitalWrite(SHOT2_PWM, 0);
  digitalWrite(ACCEPT1_PWM, 0);
  digitalWrite(ACCEPT2_PWM, 0);
  digitalWrite(PUSH_BALL, HIGH);
  digitalWrite(RELAY_STAND, HIGH);
  digitalWrite(RELAY_STRETCH, HIGH);

  // init params
  suuri.currentSpeed = 0;
  suuri.targetSpeed = 0;
  suuri.angle = 0;
  suuri.brake = true;
  suuri.rotate = NO_ROTATE;
  shot.currentSpeed = 0;
  shot.targetSpeed = 0;
  hand.angle = 0;
  hand.currentSpeed = 0;
  hand.targetSpeed = 0;
  hand.encoder = 0;
  hand.angle = 0;

  delay(1000); // wait for LPMS power on

  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_CS, PS2_DAT);  // clk, cmd, att, dat

  while (!lpms.setMode(COMMAND_MODE))
    ;
  delay(1000);
  while (!lpms.setOffset());  // euler - ийг 0 болгоно.
  
  // uncomment the following the lines for simulator of windows software.
  // if(Serial) {
  //   simulate();
  // }
}


void loop() {
  /***************** update LPMS euler ***************************/
  if (lpms.requestAngle()) {
    euler = lpms.euler[Z];
  }

  /***************** read Joystick and process commands **********/
  ps2x.read_gamepad();

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
  int rx = ps2x.Analog(PSS_RX);
  int ry = ps2x.Analog(PSS_RY);
  if ((rx < 130 && rx > 125) && (ry < 130 && ry > 125)) {
    suuri.targetSpeed = 0;
  } else {
    // joystick - ийн 0 - 255 утгыг нэмэх, хасах утга руу хөрвүүлэх
    int x = map(rx, 0, 255, -MAX_SPEED, MAX_SPEED);
    int y = map(ry, 0, 255, MAX_SPEED, -MAX_SPEED);
    suuri.targetSpeed = sqrt(x * x + y * y);
    suuri.direction = atan2(y, x) * RAD_TO_DEG;
    suuri.brake = false;
  }

  // Зүүн талын joystick
  int lx = ps2x.Analog(PSS_LX);
  int ly = ps2x.Analog(PSS_LY);
  if ((lx > 110 && lx < 140) && (ly < 140 && ly > 110)) {
    suuri.rotate = NO_ROTATE;
  } else {
    // joystick - ийн 0 - 255 утгыг нэмэх, хасах утга руу хөрвүүлэх
    int x = map(lx, 0, 255, -100, 100);
    int y = map(ly, 0, 255, 100, -100);
    Serial.print("x: ");
    Serial.print(x);
    Serial.print("\ty: ");
    Serial.println(y);

    if( x < -80)
      suuri.rotate = ROTATE_RIGHT_SLOW;  
    else if(x > 80)
      suuri.rotate = ROTATE_LEFT_SLOW;
    else if(y < -80 )
      suuri.rotate = ROTATE_RIGHT_FAST;
    else if(y > 80) 
      suuri.rotate = ROTATE_LEFT_FAST;
    suuri.brake = false;
  }

  // ps2x.Button(PSB_PAD_LEFT);                     // button дарагдаастай байвал TRUE буцаагаад л байна.
  // ps2x.ButtonPressed(unsigned int);  // RISING_EDGE шиг буюу button дарагдах мөчид нэг дуаа TRUE буцаана. Бусад тохиолдолд FALSE. Button дараастай, авах гэх мэт үед TRUE буцаахгүй гэсэн үг.
  // ps2x.ButtonReleased(unsigned int); // FALLING_EDGE шиг буюу button дараастай байдлаас гараа авах мөчид нэг удаа TRUE буцаана. Бусад тохиолдолд FALSE буцаана.

  
  if (ps2x.ButtonPressed(PSB_PAD_LEFT)) {  // Хэрвээ LEFT товчлуур дараастай байвал, роботыг зүүн тал руу эргүүлнэ.
    Serial.println("PAD LEFT");
  }
  
  if (ps2x.ButtonPressed(PSB_PAD_RIGHT)) {  // Хэрвээ RIGHT товчлуур дараастай байвал, роботыг баруун тал руу эргүүлнэ.
    Serial.println("PAD RIGHT");
  }
  if (ps2x.Button(PSB_PAD_UP)) {
    hand.currentSpeed = -50;
  }
  else if (ps2x.Button(PSB_PAD_DOWN)) {
    hand.currentSpeed = 50;
  }
  else{
    hand.currentSpeed = 0;
  }
  if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
    digitalWrite(PUSH_BALL, LOW);
    delay(2000);
    digitalWrite(PUSH_BALL, HIGH);
  }
  if (ps2x.ButtonPressed(PSB_CIRCLE)) {
    stretch = !stretch;
    digitalWrite(RELAY_STRETCH, stretch);
  }
  if (ps2x.ButtonPressed(PSB_CROSS)) {  // Хэрвээ X точвлуур дарагдвал, бүх мотор brake хийнэ.
    suuri.brake = true;
  }
  if (ps2x.ButtonPressed(PSB_SQUARE)) {
    stand = !stand;
    digitalWrite(RELAY_STAND, stand);
  }

  if (ps2x.Button(PSB_L1)) {

    shot.targetSpeed = DRIBBLE_PWM;
  }
  else if (ps2x.Button(PSB_L2)) {
    shot.targetSpeed = PASS_PWM;
  }
  else if (ps2x.Button(PSB_R1)) {
    shot.targetSpeed = SHOT_PWM;
  }
  else if (ps2x.Button(PSB_R2)) {
    shot.targetSpeed = LONG_SHOT_PWM;
  }
  else {
    shot.targetSpeed = 0;
  }

  if (ps2x.ButtonPressed(PSB_SELECT)) {
    Serial.println("SELECT pressed");
  }
  if (ps2x.ButtonPressed(PSB_START)) {
    Serial.println("START pressed");
  }

  if(suuri.brake == true) {
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
  }
  else {
    move(); // суурь явуулах
  }

  rotate(); // шидэх моторыг эргүүлэх
  pull(); // шидэх өнцөг тааруулах
}

void move() {
  // Роботыг acceleration хийж хөдөлгөхийн тулд хурд бага багаар нэмэгдүүлэх эсвэл хорогдуулна.
  if(suuri.targetSpeed == 0) {
    suuri.currentSpeed -= 2;
    if(suuri.currentSpeed > -5 && suuri.currentSpeed < 5)
      suuri.currentSpeed = 0;
  }
  else {
    if(suuri.currentSpeed < suuri.targetSpeed) {
      suuri.currentSpeed += accel;
    }
    else if(suuri.currentSpeed > suuri.targetSpeed) {
      suuri.currentSpeed -= accel;
    }
    else {
      suuri.currentSpeed = suuri.targetSpeed;
    }
  }

  float Vx = suuri.currentSpeed * cos(suuri.direction * DEG_TO_RAD);
  float Vy = suuri.currentSpeed * sin(suuri.direction * DEG_TO_RAD);

  float scale = rotate_scale;
  float theta = 0;

  if(suuri.rotate == NO_ROTATE) {
    theta = suuri.angle - euler;  // joystick - ээс ирж буй өнцөг болон LPMS өнцөг 2 - ийн зөрүүгээр робот дөөрөө эргэх хөдөлгөөн хийнэ.
  }
  else {
    suuri.angle = euler;
    if(suuri.rotate == ROTATE_LEFT_SLOW) {
      theta = -rotate_scale * 3;
    }
    else if(suuri.rotate == ROTATE_LEFT_FAST) {
      theta = -rotate_scale * 6;
    }
    else if(suuri.rotate == ROTATE_RIGHT_SLOW) {
      theta = rotate_scale * 3;
    }
    else if(suuri.rotate == ROTATE_RIGHT_FAST) {
      theta = rotate_scale * 6;
    }
  } 

  if (theta > 180) theta -= 360;
  if (theta < -180) theta += 360;

  // if(abs(theta) < 5) 
  //   scale *= 5;

  float w1 = -Vx - Vy - theta * scale;
  float w2 = -Vx + Vy - theta * scale;
  float w3 = Vx + Vy - theta * scale;
  float w4 = Vx - Vy - theta * scale;

  motor1.setSpeed(w1 * scale_m1);
  motor2.setSpeed(w2 * scale_m2);
  motor3.setSpeed(w3 * scale_m3);
  motor4.setSpeed(w4 * scale_m4);
}

void rotate() {
  if(shot.targetSpeed == 0) {
    shot.currentSpeed -= 2;
    if(shot.currentSpeed > -5 && shot.currentSpeed < 5)
      shot.currentSpeed = 0;
  }
  else {
    if(shot.currentSpeed < shot.targetSpeed) {
      shot.currentSpeed += accel;
    }
    else if(shot.currentSpeed > shot.targetSpeed) {
      shot.currentSpeed -= accel;
    }
    else {
      shot.currentSpeed = shot.targetSpeed;
    }
  }
  int pwm = constrain(shot.currentSpeed, -250, 250);

  if(pwm > 0) {
    analogWrite(SHOT1_PWM, abs(pwm));
    if(shot.targetSpeed != DRIBBLE_PWM)
      analogWrite(SHOT2_PWM, abs(pwm));
  }
  else {
    analogWrite(ACCEPT1_PWM, abs(pwm));
    analogWrite(ACCEPT2_PWM, abs(pwm));
  } 
}

void pull() {
  int pwm = constrain(hand.currentSpeed, -250, 250);
  if(hand.currentSpeed < 0){
    if(digitalRead(LIMIT_SWITCH) == HIGH) {
      digitalWrite(PULL_PWM, 0);
      digitalWrite(PUSH_PWM, 0);
      hand.currentSpeed = 0;
    }
    else {
      analogWrite(PULL_PWM, abs(pwm));
    }
  }
  else if(hand.currentSpeed > 0)
    analogWrite(PUSH_PWM, abs(pwm));
  else {
    digitalWrite(PULL_PWM, 0);
    digitalWrite(PUSH_PWM, 0);
  }
}

void encoderISR() {
  if(digitalRead(EN_B) == LOW)
    hand.encoder++;
  else
    hand.encoder--;
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
  unsigned long currentMillis = millis();
  bool start = false;
  while(1) {
    if (lpms.requestAngle()) {
      euler = lpms.euler[Z];
    }

    if(Serial.available()) {
      String inputString = Serial.readStringUntil('\n');
      // move,dir=10,speed=12,scale=0.1,accel=0.1\n
      // stop\n
      inputString.trim();
      if (inputString.startsWith("move")) {
          Serial.println(inputString);
          suuri.direction = getValue(inputString, "dir").toFloat();
          suuri.targetSpeed = getValue(inputString, "speed").toFloat();
          suuri.angle = getValue(inputString, "angle").toFloat();
          Serial.print("log->dir: "); Serial.print(suuri.direction);
          Serial.print(", speed: "); Serial.print(suuri.targetSpeed);
          Serial.print(", angle: "); Serial.println(suuri.angle);
          start = true;
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
        suuri.targetSpeed = 0;
        start = false;
      }
      else if(inputString.startsWith("rotate")) {
        shot.targetSpeed = getValue(inputString, "pwm").toInt();
      }
      else if(inputString.startsWith("unrotate")) {
        shot.targetSpeed = 0;
      }
      else if(inputString.startsWith("pull")) {
        hand.currentSpeed = getValue(inputString, "pwm").toInt();
      }
      else if(inputString.startsWith("shot")) {
        digitalWrite(PUSH_BALL, LOW);
        delay(2000);
        digitalWrite(PUSH_BALL, HIGH);
      }
    }

    Serial.print("euler=");
    Serial.print(euler);
    Serial.print(",angle=");
    Serial.print(suuri.angle);
    Serial.print(",speed=");
    Serial.print(suuri.currentSpeed);
    Serial.print(",dir=");
    Serial.println(suuri.direction);

    if(start == false)
      suuri.angle = euler;
    move();
    rotate();
    pull(); 
  }
}