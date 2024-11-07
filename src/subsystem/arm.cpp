#include "subsystem/arm.hpp"

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
    rotation->reset();
};

void Arm::setPower(float power) { motors->move(power); }

/**
 * @brief Reset the arm to its initial state
 *
 * Resets the arm's rotation sensor to zero, and sets the motor group's
 * brake mode to hold, and moves the motors to 0 power.
 */
void Arm::reset() {
    rotation->reset();
    motors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    motors->move(0);
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
void Arm::moveTo(float position, bool async) {
    if (async) {
        pros::Task task {[=, this] { moveTo(position, false); }};
    } else {
        pid.reset();
        exitCondition.reset();
        while (exitCondition.getExit() == false) {
            float error = pid.update(position - rotation->get_position());
            motors->move(error);
            exitCondition.update(error);
            pros::delay(10);
            motors->move(0);
        }
    }
}
