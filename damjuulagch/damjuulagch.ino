#include "motor.h"
#include "lpms.h"
#include "PS2X_lib.h"
#include "defines.h"

/* Моторын байрлал
     3     4
      /   \
      \   /
     2     1     
  <-- positive rotation  A = high B = low
  --> negative rotation
*/

#ifdef PASS_ROBOT
Motor motor1(4, 46, 48);  // pwm, A, B
Motor motor2(5, 52, 50);  
Motor motor3(2, 38, 40);
Motor motor4(3, 44, 42);
float rotate_scale = 3; // angle * rotate_scle => pwm
float accel = 1;  // 100 => 1 + 1 + 1 + = 100. 0 => 100 -1 - 1- 1 0
float scale_m1 = 1; //60 = 60
float scale_m2 = 1; // 60 = 60
float scale_m3 = 0.85; // 60 = 52
float scale_m4 = 1.25; // 60 = 72
#endif

#ifdef SHOT_ROBOT
Motor motor1(4, 48, 46);  // pwm, A, B
Motor motor2(5, 50, 52);  
Motor motor3(2, 40, 38);
Motor motor4(3, 44, 42);
float rotate_scale = 3; // angle * rotate_scle => pwm
float accel = 1;  // 100 => 1 + 1 + 1 + = 100. 0 => 100 -1 - 1- 1 0
float scale_m1 = 1; //60 = 60
float scale_m2 = 1; // 60 = 60
float scale_m3 = 1; // 60 = 52
float scale_m4 = 1; // 60 = 72
#endif


LPMS lpms(LPMS_SERIAL);
PS2X ps2x;

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

bool stand = false;
bool stretch = false;

void setup() {
  Serial.begin(115200); // simulator
  lpms.begin(115200);
  motor1.init();
  motor2.init();
  motor3.init();
  motor4.init();
  pinMode(PIN_EN_A, INPUT_PULLUP);
  pinMode(PIN_EN_B, INPUT_PULLUP);
  pinMode(PIN_LIMIT_SWITCH, INPUT);
  pinMode(PIN_SHOT1_PWM, OUTPUT);
  pinMode(PIN_SHOT2_PWM, OUTPUT);
  pinMode(PIN_ACCEPT1_PWM, OUTPUT);
  pinMode(PIN_ACCEPT2_PWM, OUTPUT);
  pinMode(PIN_PULL_PWM, OUTPUT);
  pinMode(PIN_PUSH_PWM, OUTPUT);
  pinMode(PIN_PUSH_BALL, OUTPUT);
  pinMode(PIN_STAND, OUTPUT);
  pinMode(PIN_STRETCH, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_EN_A), encoderISR, RISING);

  digitalWrite(PIN_PULL_PWM, 0);
  digitalWrite(PIN_PUSH_PWM, 0);
  digitalWrite(PIN_SHOT1_PWM, 0);
  digitalWrite(PIN_SHOT2_PWM, 0);
  digitalWrite(PIN_ACCEPT1_PWM, 0);
  digitalWrite(PIN_ACCEPT2_PWM, 0);
  digitalWrite(PIN_PUSH_BALL, HIGH);
  digitalWrite(PIN_STAND, HIGH);
  digitalWrite(PIN_STRETCH, HIGH);

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

  ps2x.config_gamepad(PIN_PS2_CLK, PIN_PS2_CMD, PIN_PS2_CS, PIN_PS2_DAT);  // clk, cmd, att, dat

  // analogWrite(PULL_PWM, 10);

  // while(1) {
  //   if(Serial.available()) {
  //     byte Byte = Serial.read();
  //     if(Byte == 0x30) {
  //       motor1.stop();
  //       motor2.stop();
  //       motor3.stop();
  //       motor4.stop();
  //     }
  //     else if(Byte == 0x31) {
  //       motor1.setSpeed(20);
  //     }
  //     else if(Byte == 0x32) {
  //       motor2.setSpeed(20);
  //     }
  //     else if(Byte == 0x33) {
  //       motor3.setSpeed(20);
  //     }
  //     else if(Byte == 0x34) {
  //       motor4.setSpeed(20);
  //     }
  //   }
  // }

  // while(1) {
  //   Serial.println(hand.encoder);
  //   if(Serial.available()) {
  //     byte Byte = Serial.read();
        
  //     if(Byte == 0x30) {
  //       digitalWrite(SHOT1_PWM, 0);
  //       digitalWrite(SHOT2_PWM, 0);
  //       digitalWrite(ACCEPT2_PWM, 0);
  //       digitalWrite(ACCEPT1_PWM, 0);
  //       digitalWrite(PULL_PWM, 0);
  //       digitalWrite(PUSH_PWM, 0);
  //     }
  //     else if(Byte == 0x31) {
  //       analogWrite(SHOT1_PWM, 20);
  //       digitalWrite(ACCEPT1_PWM, 0);
  //       delay(2000);
  //       analogWrite(SHOT1_PWM, 0);
  //       analogWrite(ACCEPT1_PWM, 20);
  //     }
  //     else if(Byte == 0x32) {
  //       analogWrite(SHOT2_PWM, 20);
  //       digitalWrite(ACCEPT2_PWM, 0);
  //       delay(2000);
  //       analogWrite(SHOT2_PWM, 0);
  //       analogWrite(ACCEPT2_PWM, 20);
  //     }
  //     else if(Byte == 0x35){
  //       analogWrite(PULL_PWM, 10);
  //     }
  //     else if(Byte == 0x36) {
  //       analogWrite(PUSH_PWM, 10);
  //     }
  //     else if(Byte == 0x37) {
  //       digitalWrite(PUSH_BALL, LOW);
  //       delay(5000);
  //       digitalWrite(PUSH_BALL, HIGH);
  //     }
  //   }
  // }

  // // init hand location
  // analogWrite(PULL_PWM, 100);
  // while(isLimitSwitchPressed());
  // analogWrite(PULL_PWM, 0);
  // delay(1000);
  // analogWrite(PUSH_PWM, 100);
  // while(!isLimitSwitchPressed());
  // analogWrite(PUSH_PWM, 0);
  // hand.encoder = 0;
  
  // uncomment the following the lines for simulator of windows software.
  // if(Serial) {
  //   simulate();
  // }
}


void loop() {

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
      suuri.rotate = ROTATE_RIGHT_FAST;
    else if(x > 80)
      suuri.rotate = ROTATE_LEFT_FAST;
    else if(y < -80 )
      suuri.rotate = ROTATE_RIGHT_SLOW;  
    else if(y > 80) 
      suuri.rotate = ROTATE_LEFT_SLOW;
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
    hand.currentSpeed = PUSH_PWM;
  }
  else if (ps2x.Button(PSB_PAD_DOWN)) {
    hand.currentSpeed = PULL_PWM;
  }
  else{
    hand.currentSpeed = 0;
  }
  // PUSH BALL
  if (ps2x.Button(PSB_TRIANGLE)) {
    digitalWrite(PIN_PUSH_BALL, LOW);
  }
  else {
    digitalWrite(PIN_PUSH_BALL, HIGH);
  }
  // STRETCH ROBOT
  if (ps2x.ButtonPressed(PSB_CIRCLE)) {
    stretch = !stretch;
    digitalWrite(PIN_STRETCH, stretch);
  }
  if (ps2x.ButtonPressed(PSB_CROSS)) {  // Хэрвээ X точвлуур дарагдвал, бүх мотор brake хийнэ.
    suuri.brake = true;
  }
  // STAND ROBOT
  if (ps2x.ButtonPressed(PSB_SQUARE)) {
    stand = !stand;
    digitalWrite(PIN_STAND, stand);
  }      
  // DRIBBLE BALL TO ITSELF
  if (ps2x.Button(PSB_L1)) {  // DRIBBLE
    dribble_rotate(DRIBBLE_PWM);
  }
  // PASS BALL TO SHOT ROBOT
  else if (ps2x.Button(PSB_L2)) { // PASS
    shot_rotate(PASS_PWM);
  }
  // SHOT BALL in MEDIUM range
  else if (ps2x.Button(PSB_R1)) { // SHOT medium range
    shot_rotate(SHOT_PWM);
  }
  // SHOT BALL in LONG RANGE
  else if (ps2x.Button(PSB_R2)) { // SHOT long range
    shot_rotate(LONG_SHOT_PWM);
  }
  // ACCEPT BALL AFTER DIRBBLE BALL
  else if (ps2x.Button(PSB_START)) { // ACCEPT
    accept_rotate(ACCEPT_PWM);
  }
  else {
    stop_rotate();
  }

  if (ps2x.ButtonPressed(PSB_SELECT)) {
    Serial.println("SELECT pressed");
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

  pull(); // шидэх өнцөг тааруулах
}

void move() {
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

  if(suuri.rotate != NO_ROTATE) {
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

  float w1 = -Vx - Vy - theta * scale;
  float w2 = -Vx + Vy - theta * scale;
  float w3 = Vx + Vy - theta * scale;
  float w4 = Vx - Vy - theta * scale;

  motor1.setSpeed(w1 * scale_m1);
  motor2.setSpeed(w2 * scale_m2);
  motor3.setSpeed(w3 * scale_m3);
  motor4.setSpeed(w4 * scale_m4);

}

void dribble_rotate(int speed) {
  analogWrite(PIN_SHOT1_PWM, abs(speed));
  analogWrite(PIN_SHOT2_PWM, 0);
  digitalWrite(PIN_ACCEPT1_PWM, 0);
  digitalWrite(PIN_ACCEPT2_PWM, 0);
}

void shot_rotate(int speed) {
  analogWrite(PIN_SHOT1_PWM, abs(speed));
  analogWrite(PIN_SHOT2_PWM, abs(speed));
  digitalWrite(PIN_ACCEPT1_PWM, 0);
  digitalWrite(PIN_ACCEPT2_PWM, 0);
}

void accept_rotate(int speed) {
  digitalWrite(PIN_SHOT1_PWM, 0);
  digitalWrite(PIN_SHOT2_PWM, 0);
  analogWrite(PIN_ACCEPT1_PWM, abs(speed));
  analogWrite(PIN_ACCEPT2_PWM, abs(speed));
}

void stop_rotate() {
  digitalWrite(PIN_ACCEPT1_PWM, 0);
  digitalWrite(PIN_ACCEPT2_PWM, 0);
  digitalWrite(PIN_SHOT1_PWM, 0);
  digitalWrite(PIN_SHOT2_PWM, 0);

}

// +pwm нь татна, -pwm нь буцаана.
void pull() {
  int pwm = constrain(hand.currentSpeed, -250, 250);
  if(hand.currentSpeed < 0){
    if(digitalRead(PIN_LIMIT_SWITCH) == HIGH) {
      digitalWrite(PULL_PWM, 0);
      digitalWrite(PUSH_PWM, 0);
      hand.currentSpeed = 0;
    }
    else {
      analogWrite(PIN_PUSH_PWM, abs(pwm));
    }
  }
  else if(hand.currentSpeed > 0){
    if(hand.encoder >= HAND_MAX_LOCATION)
      digitalWrite(PIN_PULL_PWM, 0);
    else
      analogWrite(PIN_PULL_PWM, abs(pwm));
  }
  else {
    digitalWrite(PIN_PULL_PWM, 0);
    digitalWrite(PIN_PUSH_PWM, 0);
  }
}

void encoderISR() {
  if(digitalRead(PIN_EN_B) == LOW)
    hand.encoder++;
  else
    hand.encoder--;
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
    // if (lpms.requestAngle()) {
    //   euler = lpms.euler[Z];
    // }

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
        int speed = getValue(inputString, "pwm").toInt();
        if(speed == 0) stop_rotate();
        else shot_rotate(speed);
      }
      else if(inputString.startsWith("unrotate")) {
        stop_rotate();
      }
      else if(inputString.startsWith("pull")) {
        hand.currentSpeed = getValue(inputString, "pwm").toInt();
      }
      else if(inputString.startsWith("shot")) {
        digitalWrite(PIN_PUSH_BALL, LOW);
        delay(2000);
        digitalWrite(PIN_PUSH_BALL, HIGH);
      }
    }

    move();
    pull(); 
  }
}