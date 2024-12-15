#include "config.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/distance.hpp"
#include "pros/rotation.hpp"
#include "subsystem/intake.hpp"

// ports
constexpr int RIGHT_F = 8;
constexpr int RIGHT_M = 20;
constexpr int RIGHT_B = -18;

constexpr int LEFT_F = -12;
constexpr int LEFT_M = -11;
constexpr int LEFT_B = 13;

constexpr int VERTI_ROT = -9;
constexpr int HOR_ROT = 16;

constexpr int INTAKE_1 = 15;
constexpr int DISTANCE = 4;

constexpr int WALLSTAKE1 = 1;
constexpr int WALLSTAKE2 = -5;
constexpr int WALLSTAKE_ROT = 19;

constexpr char CLAMP = 'A';
constexpr char HOOK = 'B';

constexpr char TOP_SORT = 3;

constexpr char IMU = 17;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drivetrain
pros::MotorGroup rightMotors({RIGHT_F, RIGHT_M, RIGHT_B}, pros::MotorGearset::blue);
pros::MotorGroup leftMotors({LEFT_F, LEFT_M, LEFT_B}, pros::MotorGearset::blue);

// intake
pros::Motor intakeMotor(INTAKE_1, pros::MotorGearset::blue);
Intake intake(intakeMotor);

// wallstake
pros::MotorGroup wallstake({WALLSTAKE1, WALLSTAKE2});
pros::Rotation wallstakeRot(WALLSTAKE_ROT);
Arm arm(&wallstake, &wallstakeRot, 10, 0, 20);

// Clamp mechanism Piston
pros::adi::DigitalOut clamp(CLAMP);

// sorting mechanism
pros::adi::DigitalOut hook(HOOK);
pros::Optical topSort(TOP_SORT);
// pros::Optical bottomSort(BOTTOM_SORT);

// Inertial Sensor on port 10
pros::Imu imu(IMU);

pros::Distance distance(DISTANCE);

pros::Rotation vertiRot(VERTI_ROT);
lemlib::TrackingWheel vertiTrackingWheel(&vertiRot, lemlib::Omniwheel::NEW_2,-1.4);

pros::Rotation horRot(HOR_ROT);
lemlib::TrackingWheel horTrackingWheel(&horRot, lemlib::Omniwheel::NEW_2,-2);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12, // 14 inch track width
                              lemlib::Omniwheel::NEW_325,
                              450, // drivetrain rpm is 450
                              5 // If you have a drift drive, we recommend starting with a value of 2, while a
                                // drivetrain with center traction wheels should start with a value of 8.
);

// lateral motion controller
lemlib::ControllerSettings linearController(16, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            115, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            10 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(7.5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             70, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertiTrackingWheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horTrackingWheel, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);