// files
#include "main.h"
#include "config.hpp"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include "subsystem/intake.hpp"
#include <iostream>

// runtime variables
bool sort = false;
double fwd;
double turning;
float up;
float down;
bool clamped = false;
bool hooked = false;
int armState = 0;
int separationState = 0;

void printTelemetry()
{
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::screen::print(TEXT_MEDIUM, 1, "x: %f", pose.x); // prints the x position
        pros::screen::print(TEXT_MEDIUM, 2, "y: %f", pose.y); // prints the y position
        pros::screen::print(TEXT_MEDIUM, 3, "theta: %f", pose.theta); // prints the heading
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Arm position: %d", arm.getPosition()); // prints the arm position

        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "left temp: %.1f %.1f %.1f", leftMotors.get_temperature(0),
            leftMotors.get_temperature(1), leftMotors.get_temperature(2));

        pros::screen::print(pros::E_TEXT_MEDIUM, 6, "right temp: %.1f %.1f %.1f", rightMotors.get_temperature(0),
            rightMotors.get_temperature(1), rightMotors.get_temperature(2));

        pros::screen::print(pros::E_TEXT_MEDIUM, 7, "intake temp: %.1f", intake.motor.get_temperature());

        std::cout << pose.x << " " << pose.y << " " << imu.get_rotation() << pose.theta << std::endl;
        switch (intake.ring) {
        case Intake::Ring::BLUE:
            controller.print(1, 1, "%s", "SEPARATING BLUE");
            break;
        case Intake::Ring::RED:
            controller.print(1, 1, "%s", "SEPARATING RED");
            break;
        case Intake::Ring::NONE:
            controller.print(1, 1, "%s", "SEPARATING NONE");
            break;
        default:
            break;
        }

        pros::delay(200);
    }
}

void init_separation(Intake::Ring ring)
{
    intakeMotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    intake.ring = ring;
    if (intake.ring == Intake::Ring::BLUE) {
        separationState = 1;
    } else if (intake.ring == Intake::Ring::RED) {
        separationState = 0;
    } else if (intake.ring == Intake::Ring::NONE) {
        separationState = 2;
    }
}

void switchSeparation()
{
    if (controller.get_digital(DIGITAL_UP) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        separationState++;
        if (separationState > 2) {
            separationState = 0;
        }
        switch (separationState) {
        case 0:
            intake.setSeparation(Intake::Ring::RED);
            break;
        case 1:
            intake.setSeparation(Intake::Ring::BLUE);
            break;
        case 2:
            intake.setSeparation(Intake::Ring::NONE);
            break;
        }
        pros::delay(500);
    }
}
void initialize()
{
    init_separation(Intake::Ring::NONE);
    chassis.calibrate(); // calibrate the chassis
    pros::Task task { [=] { intake.intakeControl(); } };
    arm.initialize();
    pros::delay(500);
    pros::Task printOdomTask(printTelemetry); // create a task to print the odometry values

}

void progSkillsWithOneWallstake() {
    wallstake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-60, 0, 90); // set the starting position of the robot
    intake.set(Intake::IntakeState::INTAKING); // start the intake
    pros::delay(600);
    chassis.moveToPoint(-46, 0, 2000,
                        {
                            .maxSpeed = 70,
                        },
                        false);
    intake.set(Intake::IntakeState::STOPPED);

    chassis.turnToHeading(0, 2000, {}, false);
    chassis.moveToPoint(-45, -32, 2000, {.forwards = false, .maxSpeed = 60}, true);
    chassis.waitUntil(23.5);
    clamp.set_value(true); // clamp the stake
    chassis.waitUntilDone();
    chassis.moveToPose(-23, -24, 90, 2000, {.minSpeed = 90}, true);
    chassis.waitUntil(5);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.turnToPoint(-20, -50, 2000, {}, false);
    chassis.moveToPoint(-20, -50, 2000, {.maxSpeed = 60}, false);
    chassis.turnToPoint(28, -52, 2000, {}, false);
    chassis.moveToPoint(28, -52, 2000, {.maxSpeed = 90}, false);
    pros::delay(250);

    chassis.moveToPoint(51, -52, 2000, {}, true);
    chassis.waitUntil(15);
    // arm.loadWallstake();
    chassis.waitUntilDone();
    pros::delay(500);
//
    // autistic ass mf of a wallstake
    chassis.moveToPoint(4.5, -45.5, 3000, {.forwards = false, .maxSpeed = 70}, false); // was 0.7
    chassis.turnToHeading(180, 3000, {.maxSpeed = 100}, false);
    chassis.moveFor(19, 3000, {.maxSpeed = 60});
    // chassis.moveToPoint(4,-54.75,1000,{},false);
    chassis.turnToHeading(180, 3000, {.maxSpeed = 100}, false);
    intake.set(Intake::IntakeState::STOPPED);
    // arm.scoreWallstake();
    // arm.retract();
    intake.set(Intake::IntakeState::INTAKING);
    pros::delay(500);
    chassis.moveFor(5, 1000, {.forwards = false}, false);

    chassis.turnToPoint(-24, -48, 2000, {}, false);
    chassis.moveToPose(-58, -55, -90, 3000, {.maxSpeed = 80}, false);
    pros::delay(100);

    chassis.moveFor(5,1000,{.forwards = false},false);
    chassis.turnToHeading(80, 2000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false);
    intake.set(Intake::IntakeState::OUTTAKE, 50);
    chassis.moveToPoint(-61, -64, 2000, {.forwards = false}, false);
    clamp.set_value(false);
    pros::delay(200);

    // get next mobile goal
    chassis.moveToPoint(-48, 0, 4000, {.maxSpeed = 65}, false);
    chassis.turnToHeading(180, 2000, {}, false);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.moveToPoint(-48, 22, 2000, {.forwards = false, .maxSpeed = 60}, false);
    chassis.waitUntil(21);
    clamp.set_value(true);
    chassis.waitUntilDone();
    pros::delay(50);

    chassis.turnToPoint(-23, 20, 2000, {}, true);
    pros::delay(300);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(-24, 20, 2000, {.minSpeed = 80, .earlyExitRange = 6}, true);
    chassis.waitUntil(3);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.waitUntil(8);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPose(3, -13, 90 + 45, 3000, {}, false);
    chassis.turnToHeading(-35, 1000, {}, false);
    chassis.moveToPose(-24, 43, 0, 2000, {}, false);
    chassis.turnToPoint(-62, 48, 1000, {.maxSpeed = 90}, false);
    pros::delay(250);
    chassis.moveToPoint(-62, 48, 1000, {.maxSpeed = 55}, false);
    pros::delay(500);
    chassis.moveToPoint(-30, 45, 1000, {.forwards = false}, false);
    chassis.moveToPoint(-52, 60, 1000, {}, false);
    pros::delay(250);

    // releasing stake
    chassis.turnToHeading(80, 1000, {}, false);
    chassis.moveToPoint(-63, 60, 1000, {.forwards = false}, false);
    clamp.set_value(false);
    intake.set(Intake::IntakeState::OUTTAKE, 30);

//score second wallstake
    chassis.moveToPoint(0, 58,3000,{},true);
    chassis.turnToHeading(0, 1000);
    chassis.moveFor(5,1000,{},false);
    // intake.set(Intake::INTAKING, 127);
    // arm.loadWallstake();
    // chassis.moveFor(6,2000,{},false);
    // pros::delay(300);
    // arm.scoreWallstake();
    // arm.retract();

    // // holding one rings in the intake
    // intake.set(Intake::IntakeState::INTAKING, 60);
    // chassis.moveToPoint(21, 20, 2000, {.maxSpeed = 70}, false);
    // pros::delay(250);
    // chassis.turnToPoint(45, -3, 2000,
    //                     {
    //                         .forwards = false,
    //                     },
    //                     true);
    // pros::delay(200);
    // intake.set(Intake::IntakeState::STOPPED);

    // // clamping new mobile goal
    // chassis.moveToPoint(48, -6, 2000, {.forwards = false, .maxSpeed = 60}, false);
    // clamp.set_value(true);
    // pros::delay(200);

    // // beginning scoring on new mobile goal
    // chassis.turnToPoint(20, 47, 2000, {}, false);

    // chassis.moveToPoint(21, 48, 2000, {.maxSpeed = 60}, true);
    // intake.set(Intake::IntakeState::INTAKING, 127);
    // chassis.turnToHeading(90, 2000, {}, false);
    // chassis.moveToPoint(48, 47, 2000, {.maxSpeed = 70}, false);

    // chassis.moveToPoint(27, 47, 2000, {.forwards = false, .maxSpeed = 100}, false);
    // chassis.turnToPoint(47, 58, 2000, {}, false);
    // chassis.moveToPoint(47, 58, 2000, {}, false);
    // chassis.turnToHeading(180 + 45, 2000, {}, false);
    // intake.set(Intake::IntakeState::OUTTAKE, 30);
    // clamp.set_value(false);
    // chassis.moveToPoint(58, 63, 1000, {.forwards = false}, false);
    // chassis.moveFor(5, 1000);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(172, 2000, {}, false);
    // chassis.moveFor(150, 4000);
    
    // // not tested
    // //  chassis.turnToPoint(48,60,2000,{},false);
    // //  chassis.turnToHeading(180+45,2000,{.direction = AngularDirection::CCW_COUNTERCLOCKWISE},false);
    // //  chassis.moveToPoint(61,60,1000,{.forwards = false},true);
    // //  intake.set(Intake::IntakeState::OUTTAKE, 30);
    // //  clamp.set_value(false);

    // // chassis.moveToPoint(37, -10, 2000, {.forwards = false, .minSpeed = 80}, false);
    // // chassis.turnToPoint(58, -47, 1000, {}, false);
    // // chassis.moveToPoint(61, -47, 1000, {}, false);
    // // chassis.turnToHeading(-45, 1000, {}, false);
    // // chassis.moveToPoint(-61, -58, 1000, {.forwards = false}, false);
    // // clamp.set_value(true);
    // // intake.set(Intake::IntakeState::OUTTAKE, 30);
    // // pros::delay(500);

    // // final rings
}

void topElim_Red()
{
    chassis.setPose(-54.7, 45, 90);

    // curve to hold first ring
    intake.set(Intake::INTAKING, 80);
    chassis.moveToPoint(-10, 47, 2000, { .maxSpeed = 70 }, false);
    pros::delay(500);
    intake.set(Intake::STOPPED);

    // aligning to stake

    chassis.turnToPoint(-33, 18, 2000, { .forwards = false }, false);
    chassis.moveToPoint(-33, 18, 4000, { .forwards = false, .maxSpeed = 70 }, true);
    chassis.waitUntil(14);
    clamp.set_value(true);
    chassis.waitUntilDone();
    intake.set(Intake::INTAKING, 127);

    // second ring
    chassis.turnToPoint(-25, 44, 2000, {}, false);
    chassis.moveFor(35, 2000, { .maxSpeed = 70 });

    // third ring
    chassis.turnToPoint(-6, 60, 2000, {}, false);

    chassis.moveToPoint(-6, 60, 2000, {}, false);

    // score allianceStake
    chassis.turnToPoint(-58, 15, 2000, {}, false);
    chassis.moveToPoint(-58, 15, 4000, { .maxSpeed = 90 }, true);
    chassis.waitUntil(14);
    arm.scoreWallstake(123, false);
    chassis.waitUntilDone();
    arm.scoreWallstake(130);
}

void topElim_Blue()
{
    chassis.setPose(54.7, 45, -90);

    // curve to hold first ring
    intake.set(Intake::INTAKING, 80);
    chassis.moveToPoint(12, 47, 2000, { .maxSpeed = 70 }, false);
    pros::delay(500);
    intake.set(Intake::STOPPED);
    chassis.moveFor(3, 2000, { .forwards = false });

    // aligning to stake

    chassis.turnToPoint(33, 18, 2000, { .forwards = false }, false);
    chassis.moveToPoint(33, 18, 4000, { .forwards = false, .maxSpeed = 70 }, true);
    chassis.waitUntil(14);
    clamp.set_value(true);
    chassis.waitUntilDone();
    intake.set(Intake::INTAKING, 127);

    // second ring
    chassis.turnToPoint(25, 44, 2000, {}, false);
    chassis.moveFor(35, 2000, { .maxSpeed = 70 });

    // third ring
    chassis.turnToPoint(6, 60, 2000, {}, false);

    chassis.moveToPoint(6, 60, 2000, {}, false);

    // score allianceStake
    chassis.turnToPoint(61, 14, 2000, {}, false);
    chassis.moveToPoint(61, 14, 4000, { .maxSpeed = 90 }, true);
    chassis.waitUntil(20);
    arm.scoreWallstake(123, false);
    chassis.waitUntilDone();
    arm.scoreWallstake(130);
}

void mogoRush_Red()
{
    // add fking code
    chassis.setPose(-58, -23, 90);

    chassis.turnToPoint(-3, -43, 1000, {}, false);
    chassis.moveToPoint(-3, -43, 4000, {}, false);
    hook.set_value(true);

    chassis.moveToPoint(-63, -19, 3000, { .forwards = false, .minSpeed = 30 }, false);
    hook.set_value(false);

    chassis.turnToHeading(180, 1000, {}, false);
    hook.set_value(false);

    // clamp stake
    chassis.moveToPoint(-22, -23, 2000, {}, true);
    chassis.waitUntil(14);
    clamp.set_value(true);

    chassis.turnToPoint(-23, -46, 2000, {}, false);
    chassis.moveToPoint(-23, -46, 2000, {}, false);

    chassis.turnToPoint(-56, -11, 2000, {}, false);
    chassis.moveToPoint(-65, -5, 2000, {}, true);
    chassis.waitUntil(20);
    arm.scoreWallstake(123);
    chassis.waitUntilDone();
    arm.scoreWallstake(130);
}

void awpRed()
{
    chassis.setPose(-56, 11, -120);
    arm.scoreWallstake(123, true);
    pros::delay(500);
    chassis.moveFor(11.8, 2000);
    chassis.waitUntilDone();
    // arm.scoreWallstake(130, true);
    // pros::delay(250);
    chassis.moveToPoint(-47, 12, 2000, { .forwards = false, .maxSpeed = 80 });
    pros::delay(500);
    arm.retract();
    chassis.waitUntilDone();
    chassis.moveToPoint(-21, 25, 2000, { .forwards = false, .maxSpeed = 70 });
    chassis.waitUntil(25);
    clamp.set_value(true);

    // first ring
    chassis.turnToPoint(-23, 40, 2000, {}, true);
    pros::delay(250);
    intake.set(Intake::IntakeState::INTAKING, 127);
    chassis.moveToPoint(-23, 44, 2000, {}, true);
    pros::delay(200);

    // second ring
    chassis.moveToPoint(-10, 43, 2000, {}, false);
    pros::delay(30);

    // third

    // releasing stake near POSITIVE CORNER
    chassis.turnToPoint(-40, 0, 2000, {}, false);
    chassis.moveToPoint(-40, 0, 3000,
        {
            .maxSpeed = 90,
            .minSpeed = 50,
            .earlyExitRange = 5,
        },
        false);
    intake.set(Intake::IntakeState::STOPPED);

    chassis.moveToPoint(-44, -42, 3000, { .maxSpeed = 70 }, true);
    chassis.waitUntil(15);
    clamp.set_value(false);

    // clamp next goal
    chassis.turnToPoint(-21, -23, 2000, { .forwards = false }, false);
    chassis.moveToPoint(-21, -23, 2000, { .forwards = false, .maxSpeed = 70 }, true);
    chassis.waitUntil(15);
    clamp.set_value(true);

    // scoring next ring
    chassis.moveToPoint(-28, -48, 2000, {}, true);
    pros::delay(500);
    intake.set(Intake::IntakeState::INTAKING, 127);

    // touching bar
    chassis.turnToPoint(-15, -28, 2000, { .minSpeed = 50 }, false);
    chassis.moveToPoint(-15, -28, 2000, { .minSpeed = 60 }, false);
}

void awpBlue()
{
    chassis.setPose(56, 11, 120);
    arm.scoreWallstake(123, true);
    pros::delay(500);
    chassis.moveFor(11.8, 2000);
    chassis.waitUntilDone();
    // arm.scoreWallstake(130, true);
    // pros::delay(250);
    chassis.moveToPoint(47, 12, 2000, { .forwards = false, .maxSpeed = 80 });
    pros::delay(500);
    arm.retract();
    chassis.waitUntilDone();
    chassis.moveToPoint(21, 25, 2000, { .forwards = false, .maxSpeed = 70 });
    chassis.waitUntil(25);
    clamp.set_value(true);

    // first ring
    chassis.turnToPoint(23, 40, 2000, {}, true);
    pros::delay(250);
    intake.set(Intake::IntakeState::INTAKING, 127);
    chassis.moveToPoint(23, 44, 2000, {}, true);
    pros::delay(200);

    // second ring
    chassis.moveToPoint(10, 43, 2000, {}, false);
    pros::delay(30);

    // third

    // releasing stake near POSITIVE CORNER
    chassis.turnToPoint(40, 0, 2000, {}, false);
    chassis.moveToPoint(40, 0, 3000,
        {
            .maxSpeed = 90,
            .minSpeed = 50,
            .earlyExitRange = 5,
        },
        false);
    intake.set(Intake::IntakeState::STOPPED);

    chassis.moveToPoint(44, -42, 3000, { .maxSpeed = 70 }, true);
    chassis.waitUntil(15);
    clamp.set_value(false);

    // clamp next goal
    chassis.turnToPoint(21, -23, 2000, { .forwards = false }, false);
    chassis.moveToPoint(21, -23, 2000, { .forwards = false, .maxSpeed = 70 }, true);
    chassis.waitUntil(15);
    clamp.set_value(true);

    // scoring next ring
    chassis.moveToPoint(28, -48, 2000, {}, true);
    pros::delay(500);
    intake.set(Intake::IntakeState::INTAKING, 127);

    // touching bar
    chassis.turnToPoint(15, -28, 2000, { .minSpeed = 50 }, false);
    chassis.moveToPoint(15, -28, 2000, { .minSpeed = 60 }, false);
}

void rightRed()
{
    chassis.setPose(-52, -54, 90);
    arm.loadWallstake(33, true);
    chassis.moveToPoint(-24, -56, 3000, { .minSpeed = 50, .earlyExitRange = 10 });

    chassis.moveToPoint(-7, -52, 2000, { .maxSpeed = 70 });
    intake.set(Intake::IntakeState::INTAKING);
    chassis.waitUntil(15);
    hook.set_value(true);
    chassis.waitUntilDone();
    chassis.moveToPoint(-20, -54, 2000, { .forwards = false });
    chassis.waitUntilDone();
    hook.set_value(false);
    chassis.moveToPoint(-12.5, -53, 2000, { .forwards = true });
    chassis.waitUntilDone();

    chassis.swingToHeading(125, lemlib::DriveSide::RIGHT, 2000);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.waitUntilDone();
    arm.scoreWallstake();
    chassis.moveToPoint(-34, -52, 2000, { .forwards = false });
    arm.retract();
    chassis.waitUntilDone();
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(-20, -43, 2000);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.turnToHeading(180, 2000);
    intake.set(Intake::IntakeState::INTAKING, 20);
    chassis.moveFor(26, 2000, { .forwards = false, .maxSpeed = 50 });
    chassis.waitUntil(23);
    clamp.set_value(true);
    chassis.waitUntilDone();
    intake.set(Intake::IntakeState::INTAKING, 127);
    pros::delay(500);
    chassis.moveToPoint(-50, -35, 2000, { .forwards = false, .maxSpeed = 80 });
    chassis.waitUntilDone();
    intake.set(Intake::IntakeState::OUTTAKE);
    clamp.set_value(false);
    pros::delay(200);
    chassis.moveToPoint(-12, -11, 3000, { .maxSpeed = 70 });
}

void rightBlue()
{
    chassis.setPose(52, -54, -90);
    // arm.loadWallstake(33, true);
    // chassis.moveToPoint(24, -56, 3000, {.minSpeed = 50, .earlyExitRange = 10});

    // chassis.moveToPoint(7, -52, 2000, {.maxSpeed = 70});
    // intake.set(Intake::IntakeState::INTAKING);
    // chassis.waitUntil(15);
    // hook.set_value(true);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(20, -54, 2000, {.forwards = false});
    // chassis.waitUntilDone();
    // hook.set_value(false);
    // chassis.moveToPoint(12.5, -53, 2000, {.forwards = true});
    // chassis.waitUntilDone();

    // chassis.swingToHeading(-125, lemlib::DriveSide::RIGHT, 2000);
    // intake.set(Intake::IntakeState::STOPPED);
    // chassis.waitUntilDone();
    // arm.scoreWallstake();
    // chassis.moveToPoint(34, -52, 2000, {.forwards = false});
    // arm.retract();
    // chassis.waitUntilDone();
    // intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(19, -43, 2000, { .maxSpeed = 70 });
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.turnToHeading(180, 2000);
    // intake.set(Intake::IntakeState::INTAKING, 30);
    chassis.moveFor(26, 2000, { .forwards = false, .maxSpeed = 45 });
    chassis.waitUntil(23);
    clamp.set_value(true);
    chassis.waitUntilDone();
    intake.set(Intake::IntakeState::INTAKING, 127);
    pros::delay(500);
    chassis.moveToPoint(50, -35, 2000, { .forwards = false, .maxSpeed = 80 });
    chassis.waitUntilDone();
    intake.set(Intake::IntakeState::OUTTAKE);
    clamp.set_value(false);
    pros::delay(200);
    // chassis.moveToPoint(12, -11, 3000, {.maxSpeed = 70});
}

void arcadeCurve(pros::controller_analog_e_t power, pros::controller_analog_e_t turn, pros::Controller mast, float f)
{
    up = mast.get_analog(power);
    down = mast.get_analog(turn);
    fwd = (exp(-f / 10) + exp((fabs(up) - 127) / 10) * (1 - exp(-f / 10))) * up;
    turning = -1 * down;
    leftMotors.move(fwd * 0.9 - turning);
    rightMotors.move(fwd * 0.9 + turning);
}

void opAsyncButtons()
{
    while (true) {
        // toggle clamp
        if (controller.get_digital(DIGITAL_R1)) {
            clamped = !clamped;
            clamp.set_value(clamped);
            pros::delay(500);
        }

        if (controller.get_digital(DIGITAL_RIGHT)) {
            hooked = !hooked;
            hook.set_value(hooked);
            pros::delay(500);
        }

        if (controller.get_digital(DIGITAL_R2)) {
            armState++;
            if (armState > 2) {
                armState = 0;
            }
            switch (armState) {
            case 0:
                arm.retract();
                break;
            case 1:
                arm.loadWallstake();
                break;
            case 2:
                intake.set(Intake::IntakeState::OUTTAKE, 50);
                arm.scoreWallstake();
                intake.set(Intake::IntakeState::STOPPED);
                break;
            default:
                break;
            }
        }

        if (controller.get_digital(DIGITAL_DOWN)) {
            arm.scoreWallstake(125);
            armState = 1;
        }

        pros::delay(10);
    }
}

void disabled() { }

void competition_initialize() { }

void autonomous()
{
    intake.setSeparation(Intake::Ring::NONE);
    progSkillsWithOneWallstake();
}

void opcontrol()
{
    intakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // arm.retract(20, true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::Task asyncButtons(opAsyncButtons);
    while (true) {
        switchSeparation();
        arcadeCurve(pros::E_CONTROLLER_ANALOG_LEFT_Y, pros::E_CONTROLLER_ANALOG_RIGHT_X, controller, 9.6);

        if (!intake.sort) {
            if (controller.get_digital(DIGITAL_L2)) // intake
            {
                intake.set(Intake::IntakeState::INTAKING);
            }
            if (controller.get_digital(DIGITAL_L1)) // outtake
            {
                intake.set(Intake::IntakeState::OUTTAKE);
            }
            if (controller.get_digital(DIGITAL_L1) == false && controller.get_digital(DIGITAL_L2) == false && controller.get_digital(DIGITAL_R2) == false) // stop intake
            {
                intake.set(Intake::IntakeState::STOPPED);
            }

            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                arm.retract();
                armState = 0;
            }

            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
                chassis.moveFor(5, 1500, { .forwards = false }, false);
            }
        }
        pros::delay(10);
    }
}