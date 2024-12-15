#include "subsystem/arm.hpp"
#include "pros/screen.hpp"

/**
 * @brief Construct a new Arm object
 *
 * @param motors The motor group that controls the arm
 * @param rotation The rotation sensor that measures the arm's angle
 * @param kP The proportional gain for the PID controller
 * @param kI The integral gain for the PID controller
 * @param kD The derivative gain for the PID controller
 * @param exitRange The range of values within which the PID will consider itself
 *                  to have reached the target
 * @param exitTime The amount of time the PID must be within the exit range before
 *                 it will consider itself to have reached the target
 */
Arm::Arm(pros::MotorGroup* motors, pros::Rotation* rotation, float kP, float kI, float kD, float exitRange,
         float exitTime)
    : motors(motors),
      rotation(rotation),
      pid(kP, kI, kD),
      exitCondition(exitRange, exitTime) {
    motors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void Arm::setPower(float power) { motors->move(power); }

/**
 * @brief Reset the arm to its initial state
 *
 * Resets the arm's rotation sensor to zero, and sets the motor group's
 * brake mode to hold, and moves the motors to 0 power.
 */
void Arm::initialize() {
    if ((rotation->get_position() / 100) > 250) {
        rotation->set_position((rotation->get_position() / 100 - 360) * 100);
    }
}

/**
 * @brief Move the arm to the specified position
 *
 * This function will run the PID controller to move the arm to the
 * specified position. If the async parameter is true, the function will
 * return immediately and the PID controller will continue to run in the
 * background. If the async parameter is false, the function will block
 * until the PID controller has finished moving the arm to the specified
 * position.
 *
 * @param position The position to move the arm to
 * @param async If true, the function will run asynchronously in the
 *              background. If false, the function will block until the
 *              PID controller has finished moving the arm.
 */
void Arm::moveTo(int position, bool async) {
    if (async) {
        pros::Task task {[=, this] { moveTo(position, false); }};
    } else {
        // reset controllers
        pid.reset(); 
        exitCondition.reset();
        // main loop
        while (exitCondition.getExit() == false) {
            int error = position - rotation->get_position() / 100; // get the error between target and current in DEGREES
            motors->move(pid.update(error)); // move the motors according to PID output
            exitCondition.update(error); // update the exit condition
            pros::delay(10); // delay to save resources 
            pros::screen::print(pros::E_TEXT_MEDIUM, 6, "error: %d", error); // prints error to screen for debugging
        }
        motors->brake(); // stops motor
    }
}

void Arm::loadWallstake(float position, bool async) {
    motors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    if (async) {
        moveTo(position, true);
    } else {
        moveTo(position, false);
    }
}

void Arm::scoreWallstake(float position, bool async) {
    motors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    if (async) {
        moveTo(position, true);
    } else {
        moveTo(position, false);
    }
}

void Arm::retract(float position, bool async) {
    motors->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    if (async) {
        moveTo(position, true);
    } else {
        moveTo(position, false);
    }
}


/**
 * @brief Get the current position of the arm
 *
 * This function returns the current position of the arm by reading the
 * rotation sensor. The position is returned in the units of the rotation
 * sensor divided by 100.
 *
 * @return The current position of the arm
 */
int Arm::getPosition() { return rotation->get_position() / 100; }
