#include <Wire.h>

const int MPU_ADDR = 0x68; // I2C address of the MPU-9250 (usually 0x68)
float gyro_sensitivity = 131.0; // For ±250°/s
float yaw = 0.0;
float gyro_bias_z = 0.0;

unsigned long previous_time;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Wake up MPU-9250
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);  // Wake up
  Wire.endTransmission(true);

  // Optional: Set gyro range to ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG
  Wire.write(0x00);  // ±250°/s
  Wire.endTransmission(true);

  delay(100);
  calibrateGyro();

  previous_time = millis();
}

void loop() {
  // Read raw gyro Z
  int16_t gz = readGyroZ();
  float gyro_z_dps = (gz - gyro_bias_z) / gyro_sensitivity;

  // Compute dt
  unsigned long current_time = millis();
  float dt = (current_time - previous_time) / 1000.0;
  previous_time = current_time;

  // Integrate yaw
  yaw += gyro_z_dps * dt;

  // Wrap yaw
  if (yaw > 180.0) yaw -= 360.0;
if (yaw < -180.0) yaw += 360.0;

  Serial.println(yaw, 2);

  delay(10);  // ~100Hz update
}

// Read raw gyro Z
int16_t readGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47); // GYRO_ZOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  int16_t gz = Wire.read() << 8 | Wire.read();
  return gz;
}

// Simple gyro Z bias calibration
void calibrateGyro() {
  long sum = 0;
  const int samples = 500;

  for (int i = 0; i < samples; i++) {
    sum += readGyroZ();
    delay(5);
  }
  gyro_bias_z = sum / (float)samples;
}