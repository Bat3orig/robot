#include "MPU9250.h"
#include "eeprom_utils.h"
#include "MadgwickAHRS.h"

MPU9250 mpu;
// Madgwick filter;

uint32_t update_ms = millis();

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    loadCalibration();

    mpu.selectFilter(QuatFilterSel::MADGWICK);
    mpu.setFilterIterations(1);
    
    // filter.begin(20);  // Sensor fusion rate (Hz)
    // delay(1000);
}

void loop() {

  if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            Serial.println(mpu.getYaw());
            prev_ms = millis();
        }
    }

  // mpu.update_accel_gyro();
  // float gx = mpu.getGyroX() * DEG_TO_RAD;
  // float gy = mpu.getGyroY() * DEG_TO_RAD;
  // float gz = mpu.getGyroZ() * DEG_TO_RAD;

  // float ax = mpu.getAccX();
  // float ay = mpu.getAccY();
  // float az = mpu.getAccZ();

  // mpu.update_mag();

  // float mx = mpu.getMagX();
  // float my = mpu.getMagY();
  // float mz = mpu.getMagZ();


  // if(millis() - update_ms > 50) {
  //   update_ms = millis();
  //   filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  //   Serial.print(filter.getYaw() * RAD_TO_DEG); 
  //   Serial.print(", ");
  //   Serial.println(filter.getYawWithCompute());
  // }
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}

void calibrateAndSave() {
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu.calibrateMag();

    print_calibration();
    mpu.verbose(false);

    // save to eeprom
    saveCalibration();

    // load from eeprom
    loadCalibration();
}