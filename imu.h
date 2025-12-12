#pragma once

#include <SparkFun_BNO080_Arduino_Library.h>

class IMU {
  public:
    double z;

    IMU();
    bool init();
    void reset();
    void update();

  private:
    BNO080 bno;
};
