#pragma once
#include "pros/motors.hpp"
#include <cmath>

class Intake {
    public:
        enum IntakeState { STOPPED, INTAKING, OUTTAKE };
        Intake(pros::Motor& motor);
        void intakeControl();
        void set(IntakeState state, int speed = 127);
        pros::Motor& motor;
        IntakeState state = STOPPED;
        int speed = 127;
};
