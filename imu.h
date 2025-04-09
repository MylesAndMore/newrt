#pragma once

#include <LSM6.h>

class IMU {
  public:
    double z;

    IMU();
    void init();
    void reset();
    void update();

  private:
    LSM6 lsm;
    int16_t offsetZ; 
    unsigned long lastUpdate;
};
