#include <PID_v1.h>
#include <Romi32U4.h>
#include <Wire.h>
#include "imu.h"

template<typename T1, typename T2>
struct Pair {
    T1 first;
    T2 second;
    Pair(T1 f, T2 s) : first(f), second(s) {}
};

typedef enum Direction {
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  END,
} Direction;

// List of sequential distance + turn pairs for the robot to make
// The robot will first go the distance (in cm), followed by making the specified turn, and then proceed to the next pair
Pair<int, Direction> turns[] = {
  {50, RIGHT},
  {50, LEFT},
  {50, LEFT},
  {50, RIGHT},
  {50, BACKWARD},
  {100, END}, // The sequence must terminate with an END turn
};
#define NUM_TURNS (sizeof(turns) / sizeof(turns[0]))
#define BASE_SPEED 50 // ~30-300

Romi32U4ButtonA button;
Romi32U4Motors motors;
Romi32U4Encoders encoders;
IMU imu;

#define COUNTS_PER_REV 1437.09 // See https://pololu.github.io/romi-32u4-arduino-library/class_romi32_u4_encoders.html
#define WHEEL_CIRCUMFERENCE_M M_PI * 7 // Romi wheels are 70mm in diameter
int64_t totalCountsL = 0, totalCountsR = 0;

#define DRIVE_KP 4
#define DRIVE_KI 0
#define DRIVE_KD 0.35
double pidOut, pidSet;
PID pid(&imu.z, &pidOut, &pidSet, DRIVE_KP, DRIVE_KI, DRIVE_KD, DIRECT);

int currentTurn = 0;

float get_dist_traveled() {
  // Accumulate counts in 64-bit integers to prevent overflow
  totalCountsL += encoders.getCountsAndResetLeft();
  totalCountsR += encoders.getCountsAndResetRight();
  float totalCounts = (totalCountsL + totalCountsR) / 2; // Average of left and right encoders to increase accuracy
  return (totalCounts / COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE_M;
}

void reset_dist() {
  totalCountsL = 0;
  totalCountsR = 0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // Delay so gyro is calibrated when still
  delay(500);
  encoders.init();
  imu.init();
  pid.SetOutputLimits(-150, 150); // Half of motor full range
  pid.SetMode(AUTOMATIC);
  // Turn LED on to indicate that all initialization is complete
  ledYellow(1);
  // Wait until button is presed to exit setup and begin routine
  while (!button.getSingleDebouncedRelease());
  imu.reset();
}

void loop() {
  // Compute deviation from desired to percieved degree value
  imu.update();
  pid.Compute();
  // Apply control signal to drivetrain
  int16_t speedL = BASE_SPEED - pidOut;
  int16_t speedR = BASE_SPEED + pidOut;
  motors.setSpeeds(speedL, speedR);
  // Turn if necessary
  if (get_dist_traveled() >= turns[currentTurn].first) {
    if (currentTurn > NUM_TURNS) {
      motors.setSpeeds(0, 0);
      while (true);
    }
    if (turns[currentTurn].second == LEFT) {
      pidSet += 90;
    } else if (turns[currentTurn].second == RIGHT) {
      pidSet -= 90;
    } else if (turns[currentTurn].second == BACKWARD) {
      if (pidSet >= 180) {
        pidSet += 180;
      } else {
        pidSet -= 180;
      }
    }
    reset_dist();
    currentTurn++;
  }
}
