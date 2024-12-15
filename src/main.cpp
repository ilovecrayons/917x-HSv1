#include "main.h"
#include "config.hpp"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include <fstream>
#include <iostream>
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/screen.hpp"
#include "subsystem/intake.hpp"

// runtime variables
bool sort = false;
double fwd;
double turning;
float up;
float down;
bool clamped = false;
int armState = 0;

void coutPosition(){
    lemlib::Pose pose = chassis.getPose();
    std::cout << pose.x << " " << pose.y << " " << pose.theta << std::endl;
}

void printTelemetry() {
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::screen::print(TEXT_MEDIUM, 1, "x: %f", pose.x); // prints the x position
        pros::screen::print(TEXT_MEDIUM, 2, "y: %f", pose.y); // prints the y position
        pros::screen::print(TEXT_MEDIUM, 3, "theta: %f", pose.theta); // prints the heading
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Arm position: %d", arm.getPosition()); // prints the arm position
        pros::delay(250);
        coutPosition();
    }
}


void initialize() {
	chassis.calibrate(); // calibrate the chassis
	pros::Task printOdomTask(printTelemetry); // create a task to print the odometry values
	pros::Task task {[=] { intake.intakeControl(); }};
	intakeMotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	arm.initialize();
}


void disabled() {}

void progSkillsWithOneWallstake(){
    wallstake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-60, 0, 90); // set the starting position of the robot
    intake.set(Intake::IntakeState::INTAKING); // start the intake
    pros::delay(750);
    chassis.moveToPoint(-47,0,2000,
                        {
                            .maxSpeed = 70,
                        },
                        false);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.turnToHeading(0, 2000, {}, false);
    chassis.moveToPoint(-48, -32, 2000, {.forwards = false, .maxSpeed = 70}, true);
    chassis.waitUntil(24.5);
    clamp.set_value(true); // clamp the stake
    chassis.waitUntilDone();
    pros::delay(350);
    chassis.moveToPose(-23, -24, 90, 2000, {.minSpeed = 90}, true);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.turnToPoint(-23, -50, 2000, {}, false);
    chassis.moveToPoint(-23, -50, 2000, {.maxSpeed = 80}, false);
    chassis.turnToPoint(28, -49, 2000, {}, false);
    chassis.moveToPoint(28, -49, 2000, {.maxSpeed = 80}, false);
    pros::delay(250);  
    
    chassis.moveToPoint(55, -50, 2000, {.maxSpeed = 70},true);
    coutPosition();

    chassis.waitUntil(10);
    arm.loadWallstake();

    chassis.moveToPoint(9.5,-49,3000,{.forwards = false,.maxSpeed = 70},false);
    chassis.turnToHeading(180,3000,{.maxSpeed = 100},false);
    chassis.moveFor(7,1000);
    // chassis.moveToPoint(4,-54.75,1000,{},false);
    chassis.turnToHeading(180,2000,{},false);
    intake.set(Intake::IntakeState::STOPPED);
    arm.scoreWallstake();
    
    arm.retract(10,true);
    intake.set(Intake::IntakeState::INTAKING, 127);
    chassis.moveFor(5, 1000,{},false);
    chassis.moveFor(5,1000,{.forwards = false},false);


    pros::delay(500);
    chassis.turnToPoint(-24, -54, 2000, {}, false);
    chassis.moveToPose(-56, -54, -90, 2000, {.maxSpeed = 80}, false);
    pros::delay(250);
    chassis.moveToPoint(-50,-54,2000,{.forwards = false,.minSpeed = 90},false);
    chassis.turnToHeading(20, 2000, {}, false);
    intake.set(Intake::IntakeState::OUTTAKE, 50);
    chassis.moveToPoint(-58, -63, 2000, {.forwards = false}, false);
    clamp.set_value(false);
    pros::delay(200);


//get next mobile goal
    chassis.moveToPoint(-50.8, 0, 3000, {.maxSpeed = 70}, false);
    chassis.turnToHeading(180, 2000, {}, false);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.moveToPoint(-50.2, 22, 2000, {.forwards = false, .maxSpeed = 60}, false);
    chassis.waitUntil(23);
    clamp.set_value(true);
    chassis.waitUntilDone();
    pros::delay(500);
    intake.set(Intake::IntakeState::INTAKING);

    chassis.turnToPoint(-23, 20, 2000, {}, false);
    chassis.moveToPoint(-24, 20, 2000, {.minSpeed = 90, .earlyExitRange = 6}, true);
    chassis.moveToPose(-1, 0, 90 + 45, 3000, {}, false);
    chassis.waitUntil(-19);
    // chassis.setPose(-3, -4, 90 + 45);
    pros::delay(750);
    chassis.turnToHeading(-45, 1000, {}, false);
    chassis.moveToPose(-26, 50, 0, 3000, {}, false);
    chassis.turnToPoint(-62, 52, 1000, {}, false);
    chassis.moveToPoint(-62, 52, 1000, {.maxSpeed = 80}, false);
    pros::delay(500);
    chassis.moveToPoint(-34, 50, 1000, {.forwards = false}, false);
    chassis.moveToPoint(-50, 60, 1000, {}, false);
    pros::delay(500);

    //releasing stake
    chassis.turnToHeading(80, 1000, {}, false);
    chassis.moveToPoint(-63, 65, 1000, {.forwards = false}, false);
    clamp.set_value(false);
    intake.set(Intake::IntakeState::OUTTAKE, 70);

    chassis.moveToPoint(0, 55, 2000, {.maxSpeed = 90,.minSpeed = 30, .earlyExitRange = 6}, false);

    //holding one rings in the intake
    intake.set(Intake::IntakeState::INTAKING, 30);
    chassis.moveToPoint(22, 24, 2000, {.maxSpeed = 80}, false);
    pros::delay(500);
    chassis.turnToPoint(49, 0, 2000, {.forwards = false}, false);

    //clamping new mobile goal
    chassis.moveToPoint(50.8, 0, 2000, {.forwards = false, .maxSpeed = 70}, true);
    chassis.waitUntil(22);  
    clamp.set_value(true);  
    pros::delay(500);
    intake.set(Intake::IntakeState::INTAKING, 127);

    //beginning scoring on new mobile goal
    chassis.turnToPoint(20,53,2000,{},false);
    chassis.moveToPoint(20,53,2000,{ .maxSpeed = 60 },false);
    chassis.turnToHeading(90,2000,{},false);
    chassis.moveToPoint(48,54,2000,{.maxSpeed = 90},false);

    chassis.moveToPoint(30,54,2000,{.forwards = false,.maxSpeed = 100},false);
    chassis.turnToPoint(49,67,2000,{},false);
    chassis.moveToPoint(49,67,2000,{},false);
    chassis.turnToHeading(180+45,2000,{},false);
    chassis.moveToPoint(52,67,1000,{.forwards = false},false);
    intake.set(Intake::IntakeState::OUTTAKE, 30);
    clamp.set_value(false);
    chassis.moveToPose(42,0,90+45,2000,{.minSpeed = 40,.earlyExitRange = 10},false);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.moveToPoint(63,-24,2000,{.minSpeed = 40, .earlyExitRange = 8},false);
    chassis.moveToPoint(63,-58,2000,{.maxSpeed = 90},false);
    // chassis.turnToPoint(48,60,2000,{},false);
    // chassis.turnToHeading(180+45,2000,{.direction = AngularDirection::CCW_COUNTERCLOCKWISE},false);
    // chassis.moveToPoint(61,60,1000,{.forwards = false},true);
    // intake.set(Intake::IntakeState::OUTTAKE, 30);
    // clamp.set_value(false);

    // chassis.moveToPoint(37, -10, 2000, {.forwards = false, .minSpeed = 80}, false);
    // chassis.turnToPoint(58, -47, 1000, {}, false);
    // chassis.moveToPoint(61, -47, 1000, {}, false);
    // chassis.turnToHeading(-45, 1000, {}, false);
    // chassis.moveToPoint(-61, -58, 1000, {.forwards = false}, false);
    // clamp.set_value(true);
    // intake.set(Intake::IntakeState::OUTTAKE, 30);
    // pros::delay(500);

    //final rings
}

void competition_initialize() {}


void autonomous() {

}


void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
}