#include "imu.h"

#define OFFSET_SAMPLES 10000 // Samples for static gyro drift offset

IMU::IMU() : z(0.0) {}

void IMU::init() {
  lsm.init();
  lsm.enableDefault();
  lsm.writeReg(LSM6::CTRL2_G, 0b10001000); // Full scale 1000dps
  // Find Z offset
  int64_t total = 0;
  for (uint32_t i = 0; i < OFFSET_SAMPLES; i++) {
    // Wait for new data
    while((!lsm.readReg(LSM6::STATUS_REG)) & 0x08);
    lsm.read();
    total += lsm.g.z;
  }
  offsetZ = total / OFFSET_SAMPLES;
  Serial.print("offsetZ: ");
  Serial.println(offsetZ);
  reset();
}

void IMU::reset() {
  z = 0.0;
  lastUpdate = micros();
}

void IMU::update() {
  lsm.readGyro();
  int16_t rateZ = lsm.g.z - offsetZ;
  unsigned long t = micros();
  float dt = (t - lastUpdate) / 1e6; // Delta-time in seconds
  lastUpdate = t;
  // Convert raw rate to degrees per second
  float dps = rateZ * 0.035; // 0.035 dps/digit
  // Integrate to get change in angle (degrees)
  z += dps * dt;
}
