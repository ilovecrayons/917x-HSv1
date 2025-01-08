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

/**
 * @brief Filter a vector of distances using the IQR method.
 *
 * This function works by first sorting the vector of distances, then finding the
 * first quartile (Q1) and third quartile (Q3) of the sorted vector. The interquartile
 * range (IQR) is then calculated as Q3 - Q1. The upper and lower thresholds are
 * calculated as Q3 + 1.5 * IQR and Q1 - 1.5 * IQR, respectively. All distances in
 * the vector that are within the range [lowerThreshold, upperThreshold] are added to
 * a new vector, which is then averaged and returned as the filtered distance.
 *
 * @param distances The vector of distances to be filtered.
 * @return The filtered distance.
 */
float lemlib::Chassis::filterDistance(std::vector<float> distances) {
    std::sort(distances.begin(), distances.end());
    float Q1 = distances[distances.size() / 4];
    float Q3 = distances[distances.size() * 3 / 4];
    float IQR = Q3 - Q1;
    float upperThreshold = Q3 + 1.5 * IQR;
    float lowerThreshold = Q1 - 1.5 * IQR;
    std::vector<float> filteredDistances;
    for (int i = 0; i < distances.size(); i++) {
        if (distances[i] > lowerThreshold && distances[i] < upperThreshold) {
            filteredDistances.push_back(distances[i]);
        }
    }
    return lemlib::avg(filteredDistances);
}

/**
 * @brief Collects distance measurements from a distance sensor.
 *
 * This function continuously collects distance measurements from the given
 * distance sensor and stores them in the collectedDistances vector. The measurements
 * are converted from millimeters to inches by dividing by 25.4. The function runs
 * in a loop until the inDistanceCollection flag is set to false.
 *
 * @param distanceSensor The distance sensor from which measurements are collected.
 */
void lemlib::Chassis::collectDistances(pros::Distance& distanceSensor) {
    this->inDistanceCollection = true; // indicate that distance collection is running. 
    this->collectedDistances.clear();
    while (this->inDistanceCollection) { // class field can be modified externally to end distance collection
        this->collectedDistances.push_back(distanceSensor.get_distance() / 25.4); // convert from cm to in
        pros::delay(25);
    }
}

void lemlib::Chassis::endCollectDistances() { this->inDistanceCollection = false; }