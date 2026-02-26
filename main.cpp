#include "vex.h"

using namespace vex;

brain Brain;

// -------------------- Motors --------------------
// Left side
motor L1(PORT7, ratio18_1, false);
motor L2(PORT8, ratio18_1, false);

// Right side
motor R1(PORT10, ratio18_1, true);
motor R2(PORT11, ratio18_1, true);

// -------------------- Functions --------------------
void setDrive(double power) {
  L1.spin(fwd, power, pct);
  L2.spin(fwd, power, pct);

  R1.spin(fwd, power, pct);
  R2.spin(fwd, power, pct);
}

void resetEncoders() {
  L1.resetPosition();
  L2.resetPosition();

  R1.resetPosition();
  R2.resetPosition();
}

double getAverageEncoder() {
  return (L1.position(deg) + L2.position(deg) +
          R1.position(deg) + R2.position(deg)) / 4.0;
}

// -------------------- PID Drive --------------------
void drivePID(double targetDeg) {
  // ===== PID constants =====
  double kP = 0.35;
  double kI = 0.0002;
  double kD = 0.6;

  double error = 0;
  double prevError = 0;
  double integral = 0;
  double derivative = 0;

  resetEncoders();

  while (true) {
    double position = getAverageEncoder();
    error = targetDeg - position;

    // Anti-windup
    if (fabs(error) < 50)
      integral += error;
    else
      integral = 0;

    derivative = error - prevError;

    double power = (kP * error) + (kI * integral) + (kD * derivative);

    // Slow-down near target
    double maxPower = 100;
    if (fabs(error) < 150) maxPower = 40;
    if (fabs(error) < 60)  maxPower = 25;

    // Clamp motor power
    if (power > maxPower) power = maxPower;
    if (power < -maxPower) power = -maxPower;

    setDrive(power);
    prevError = error;

    // Stop condition (no overshoot)
    if (fabs(error) < 5 && fabs(derivative) < 1)
      break;

    wait(20, msec);
  }

  // Stop motors with hold to prevent rebound
  L1.stop(hold);
  L2.stop(hold);
  R1.stop(hold);
  R2.stop(hold);
}

// -------------------- Main --------------------
int main() {

  // Forward
  drivePID(720);   // 2 wheel rotations
  wait(1, sec);

  // Backward
  drivePID(-720);

  while (true) {
    wait(100, msec);
  }
}
