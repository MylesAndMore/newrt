#include <Wire.h>

#include "imu.h"

#define IMU_ADDR 0x4B // Devboard we're using uses the high address, other boards usually use 0x4A

IMU::IMU() : z(0.0) {}

bool IMU::init() {
  if (!bno.begin(IMU_ADDR)) {
    return false;
  }
  // Increase data rate
  Wire.setClock(400000);
  bno.enableRotationVector(50); // Send data every 50ms
  // Wait until we have a stable data stream to reset
  while (!bno.dataAvailable()) { delay(5); }
  reset();
  return true;
}

void IMU::reset() {
  z = 0.0;
  bno.tareNow(true); // Reset Z axis
  delay(50);
}

void IMU::update() {
  if (!bno.dataAvailable()) {
    return;
  }
  z = degrees(bno.getYaw());
}
