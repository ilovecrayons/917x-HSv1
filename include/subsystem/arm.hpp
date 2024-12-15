#pragma once
#include "lemlib/pid.hpp"
#include "lemlib/exitcondition.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

class Arm {
    public:
        Arm(pros::MotorGroup* motors, pros::Rotation* rotation, float kP, float kI, float kD, float exitRange = 10,
            float exitTime = 150);

        void setPower(float power);
        void initialize();
        void moveTo(int position, bool async = false);
        void loadWallstake(float position = 33, bool async = false);
        void scoreWallstake(float position = 164, bool async = false);
        void retract(float position = 10, bool async = false);
        int getPosition();
    private:
        pros::Rotation* rotation;
        pros::MotorGroup* motors;
        lemlib::PID pid;
        lemlib::ExitCondition exitCondition;
        int state = 0;
};