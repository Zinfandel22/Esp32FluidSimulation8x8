#include "utils.h"

namespace utils {


// initialize qmi8658 accelerometer sensor
bool initQMI8658() {
  uint8_t whoami = readRegister(QMI8658_WHO_AM_I);

  // expected device id is 0x05
  if (whoami != 0x05) {
    return false;
  }

  // configure accelerometer
  // ctrl1: enable accelerometer
  writeRegister(QMI8658_CTRL1, 0x60);
  delay(10);

  // ctrl2: accelerometer settings (4g range, 1000Hz ODR)
  writeRegister(QMI8658_CTRL2, 0x35);

  // ctrl3: gyroscope settings
  //writeRegister(QMI8658_CTRL3, 0x45); only gyro used

  // ctrl7: enable sensors
  writeRegister(QMI8658_CTRL7, 0x01); // accel only

  delay(100);
  return true;
}

// read accelerometer data in g's (1g = 9.81 m/s^2)
void readQMI8658Accel(float* x, float* y, float* z) {
  Wire.beginTransmission(QMI8658_ADDR);
  Wire.write(QMI8658_ACC_X_L);
  Wire.endTransmission(false);
  Wire.requestFrom(QMI8658_ADDR, 6);

  if (Wire.available() >= 6) {
    // read 16-bit values for each axis (little endian)
    int16_t rawX = Wire.read() | (Wire.read() << 8);
    int16_t rawY = Wire.read() | (Wire.read() << 8);
    int16_t rawZ = Wire.read() | (Wire.read() << 8);

    // convert raw values to g's
    // for 4g range: sensitivity is 8192 LSB/g
    *x = rawX / 8192.0;
    *y = rawY / 8192.0;
    *z = rawZ / 8192.0;
  }
}

// write value to sensor register
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(QMI8658_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// read value from sensor register
uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(QMI8658_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(QMI8658_ADDR, 1);

  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

}  // namespace utils