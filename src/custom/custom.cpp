#include <cmath>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "pros/distance.hpp"
#include "pros/misc.hpp"

/**
 * @brief Moves the robot by a certain distance in a certain direction.
 * 
 * @param distance The distance to move the robot by.
 * @param timeout The time in milliseconds to allow the function to complete.
 * @param params The parameters for the function.
 * @param async Whether or not the function should be run asynchronously.
 * 
 * @details The function moves the robot by a certain distance in a certain direction. It
 * uses the lateral PID and angular PID to control the movement of the robot. If the
 * function is run asynchronously, it will return immediately and the function will be
 * run in a separate task. The function will also automatically stop the drivetrain when
 * it is finished.
 * 
 * @note The function will not block other tasks in the system. If the robot is already
 * moving when the function is called, the function will not move the robot until the
 * previous motion has finished.
 * 
 * @see lemlib::Chassis::requestMotionStart()
 * @see lemlib::Chassis::endMotion()
 * @see lemlib::Chassis::moveTo()
 * @see lemlib::Chassis::turnTo()
 * @see lemlib::Chassis::moveToPose()
 * @see lemlib::Chassis::turnToPoint()
 */
void lemlib::Chassis::moveFor(float distance, int timeout, MoveForParams params, bool async) {
    params.earlyExitRange = fabs(params.earlyExitRange);
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { moveFor(distance, timeout, params, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    // reset PIDs and exit conditions
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();

    // initialize vars used between iterations
    Pose startingPose = getPose(false);
    float targetAngle = startingPose.theta;
    distTraveled = 0;
    Timer timer(timeout);
    bool close = false;
    float prevLateralOut = 0; // previous lateral power
    float prevAngularOut = 0; // previous angular power
    const int compState = pros::competition::get_status();
    std::optional<bool> prevSide = std::nullopt;
    
    // main loop
    while (!timer.isDone() && ((!lateralSmallExit.getExit() && !lateralLargeExit.getExit()) || !close) &&
           this->motionRunning) {
        // update variables
        const Pose pose = getPose(false);

        // update distance traveled
        distTraveled = pose.distance(startingPose);

        // calculate distnace to the target
        const float distTarget = distance - distTraveled;
        // check if the robot is close enough to the target to start settling
        if (distTarget < 7.5 && !close) {
            close = true;
            params.maxSpeed = fmax(fabs(prevLateralOut), 60); // slows down robot
        }


        // calculate error
        const float angularError = targetAngle - pose.theta;

        lateralSmallExit.update(distTarget);
        lateralLargeExit.update(distTarget);

        // get output for PIDs
        float lateralOut = lateralPID.update(distTarget);

        float angularOut = angularPID.update(angularError);
        if (close) angularOut = 0;

        // apply restrictions on angular speed
        angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);
        angularOut = slew(angularOut, prevAngularOut, angularSettings.slew);

        // apply restrictions on lateral speed
        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);

        // constrain lateral output by max accel, but not for deceleration
        if (!close) lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);

        // // prevent moving in the wrong direction
        // if (params.forwards && !close) lateralOut = std::fmax(lateralOut, 0);
        // else if (!params.forwards && !close) lateralOut = std::fmin(lateralOut, 0);

        // constrain lateral output by the minimum speed
        if (params.forwards && lateralOut < fabs(params.minSpeed) && lateralOut > 0) lateralOut = fabs(params.minSpeed);
        if (!params.forwards && -lateralOut < fabs(params.minSpeed) && lateralOut < 0)
            lateralOut = -fabs(params.minSpeed);

        // update previous output
        prevAngularOut = angularOut;
        prevLateralOut = lateralOut;

        infoSink()->debug("Angular Out: {}, Lateral Out: {}", angularOut, lateralOut);

        // apply direction
        if(!params.forwards) lateralOut = -lateralOut;

        // ratio the speeds to respect the max speed
        float leftPower = lateralOut + angularOut * 1;
        float rightPower = lateralOut - angularOut * 1;
        const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

        
        // move the drivetrain
        drivetrain.leftMotors->move(leftPower);
        drivetrain.rightMotors->move(rightPower);

        // delay to save resources
        pros::delay(10);
    }

    // stop the drivetrain
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->endMotion();
}
