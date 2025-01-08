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

constexpr int VERTI_ROT = 6;
constexpr int HORI_ROT = -5;

constexpr int INTAKE_1 = 15;
constexpr int DISTANCE = 16;

constexpr int WALLSTAKE1 = -1;
constexpr int WALLSTAKE2 = 4;
constexpr int WALLSTAKE_ROT = 3;

constexpr char CLAMP = 'A';
constexpr char HOOK = 'B';
constexpr char HANG = 'G';

constexpr char TOP_SORT = 10;

constexpr char IMU = 21;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// optical
pros::Optical topSort(TOP_SORT);

// drivetrain
pros::MotorGroup rightMotors({RIGHT_F, RIGHT_M, RIGHT_B}, pros::MotorGearset::blue);
pros::MotorGroup leftMotors({LEFT_F, LEFT_M, LEFT_B}, pros::MotorGearset::blue);

// wallstake
pros::MotorGroup wallstake({WALLSTAKE1, WALLSTAKE2});
pros::Rotation wallstakeRot(WALLSTAKE_ROT);
Arm arm(&wallstake, &wallstakeRot, 10, 0, 20);

// intake
pros::Motor intakeMotor(INTAKE_1, pros::MotorGearset::blue);
Intake intake(intakeMotor, topSort, arm);



// Clamp mechanism Piston
pros::adi::DigitalOut clamp(CLAMP);

// hang piston
pros::adi::DigitalOut hang(HANG); 

// sorting mechanism
pros::adi::DigitalOut hook(HOOK);
// pros::Optical bottomSort(BOTTOM_SORT);

// Inertial Sensor 
pros::Imu imu(IMU);

pros::Distance distance(DISTANCE);

pros::Rotation vertiRot(VERTI_ROT);
pros::Rotation horiRot(HORI_ROT);
lemlib::TrackingWheel vertiTrackingWheel(&vertiRot, lemlib::Omniwheel::NEW_2, -1.5);
lemlib::TrackingWheel horiTrackingWheel(&horiRot, lemlib::Omniwheel::NEW_2, -1.6,-1);

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
                                            6 // maximum acceleration (slew)
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
                            &horiTrackingWheel, // horizontal tracking wheel
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