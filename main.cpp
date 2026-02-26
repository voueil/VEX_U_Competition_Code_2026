#include "vex.h"

using namespace vex;

// Brain & Controller
brain Brain;
controller Controller1(primary);
competition Competition;

// =====================
// Sensors
// =====================
inertial InertialSensor(PORT5);
optical opti(PORT6);
digital_out DigitalOutA(Brain.ThreeWirePort.A);

// =====================
// Drive Motors
// =====================
motor l1(PORT2, ratio18_1, false);
motor l2(PORT4, ratio18_1, true);
motor_group LeftDrive(l1, l2);

motor r1(PORT1, ratio18_1, true);
motor r2(PORT3, ratio18_1, false);
motor_group RightDrive(r1, r2);

// =====================
// Intake Motors
// =====================
motor Motor11(PORT7, ratio18_1, false);
motor Motor12(PORT8, ratio18_1, false);
motor Motor13(PORT9, ratio18_1, false);
motor Motor14(PORT10, ratio18_1, false);
motor Motor15(PORT11, ratio18_1, false);

// =====================
// PID Constants
// =====================
double kP_drive = 0.02, kI_drive = 0, kD_drive = 0;
double kP_turn = 0.13, kI_turn = 0, kD_turn = 0;

double gearRatio = 1.0;
double wheelDiameter = 3.75;

// =====================
// Helper
// =====================
double inchesToDegrees(double inches) {
  double wheelCircumference = wheelDiameter * 3.14159265;
  return (inches / wheelCircumference) * 360.0 / gearRatio;
}

// =====================
// Drive PID
// =====================
void drivePID(double targetInches, int timeout = 3000) {

  double targetDegrees = inchesToDegrees(targetInches);
  double error, lastError = 0, derivative, integral = 0;
  double startTime = Brain.timer(msec);

  LeftDrive.setPosition(0, degrees);
  RightDrive.setPosition(0, degrees);

  while (true) {

    double avgPosition =
      (LeftDrive.position(degrees) + RightDrive.position(degrees)) / 2;

    error = targetDegrees - avgPosition;
    derivative = error - lastError;
    integral += error;

    double power = kP_drive * error +
                   kI_drive * integral +
                   kD_drive * derivative;

    LeftDrive.spin(forward, power, voltageUnits::volt);
    RightDrive.spin(forward, power, voltageUnits::volt);

    lastError = error;

    if (fabs(error) < 5 ||
        Brain.timer(msec) - startTime > timeout)
      break;

    wait(20, msec);
  }

  LeftDrive.stop(brake);
  RightDrive.stop(brake);
}

// =====================
// Turn PID
// =====================
void turnPID(double targetAngle, int timeout = 2000) {

  InertialSensor.setRotation(0, degrees);

  double error, lastError = 0, derivative, integral = 0;
  double startTime = Brain.timer(msec);

  while (true) {

    double currentAngle = InertialSensor.rotation();
    error = targetAngle - currentAngle;

    derivative = error - lastError;
    integral += error;

    double power = kP_turn * error +
                   kI_turn * integral +
                   kD_turn * derivative;

    LeftDrive.spin(forward, power, volt);
    RightDrive.spin(reverse, power, volt);

    lastError = error;

    if (fabs(error) < 2 ||
        Brain.timer(msec) - startTime > timeout)
      break;

    wait(20, msec);
  }

  LeftDrive.stop(brake);
  RightDrive.stop(brake);
}

// =====================
// Pre-auton
// =====================
void pre_auton(void) {
  vexcodeInit();
  InertialSensor.calibrate();
  while (InertialSensor.isCalibrating())
    wait(20, msec);
}

// =====================
// Autonomous
// =====================
void autonomous(void) {
  drivePID(24);
  wait(300, msec);
  turnPID(90);
}

// =====================
// User Control
// =====================
void usercontrol(void) {

  while (true) {

    // Arcade Drive
    int forward = Controller1.Axis3.position();
    int turn    = Controller1.Axis1.position();

    LeftDrive.spin(fwd, forward + turn, pct);
    RightDrive.spin(fwd, forward - turn, pct);

    // Optical light
    opti.setLight(ledState::on);
    opti.setLightPower(100, pct);

    // Intake Control
    if (Controller1.ButtonL1.pressing()) {

      Motor11.spin(fwd,100,pct);
      Motor12.spin(fwd,100,pct);
      Motor13.spin(fwd,100,pct);
      Motor14.spin(fwd,100,pct);
      Motor15.spin(fwd,100,pct);

    }
    else if (Controller1.ButtonL2.pressing()) {

      Motor11.spin(reverse,100,pct);
      Motor12.spin(reverse,100,pct);
      Motor13.spin(reverse,100,pct);
      Motor14.spin(reverse,100,pct);
      Motor15.spin(reverse,100,pct);

    }
    else {

      Motor11.stop(coast);
      Motor12.stop(coast);
      Motor13.stop(coast);
      Motor14.stop(coast);
      Motor15.stop(coast);
    }

    // Pneumatic
    DigitalOutA.set(Controller1.ButtonA.pressing());

    wait(20, msec);
  }
}

// =====================
// Main
// =====================
int main() {

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true)
    wait(100, msec);
}
