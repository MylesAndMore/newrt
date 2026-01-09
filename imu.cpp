// SENSOR CALIBRATION PROCEDURE: https://docs.sparkfun.com/SparkFun_VR_IMU_Breakout_BNO086_QWIIC/assets/component_documentation/BNO080-BNO085-Sesnor-Calibration-Procedure.pdf

#include <Wire.h>

#include "imu.h"

#define IMU_ADDR 0x4B // Devboard we're using uses the high address, other boards usually use 0x4A

IMU::IMU() : z(0.0) {}

bool IMU::init() {
  bno.enableDebugging();
  Wire.flush();
  if (!bno.begin(IMU_ADDR)) {
    return false;
  }
  // Increase data rate
  Wire.setClock(400000);
  bno.enableRotationVector(50); // Send data every 50ms
  // Wait until we have a stable data stream to reset
  Serial.print("waiting for data");
  while (!bno.dataAvailable()) {
    Serial.print(".");
    delay(500); 
  }
  Serial.println();
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
