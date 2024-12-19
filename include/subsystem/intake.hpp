#pragma once
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include <cmath>

class Intake {
    public:

        Intake( pros::Motor& motor , pros::Optical& topSort);

        enum IntakeState{ STOPPED , INTAKING , OUTTAKE };
        enum Ring { BLUE , RED , NONE };

        void intakeControl();
        void set(IntakeState state, int speed = 127);
        void setSeparation(Ring ring);
        void checkForSort();

        pros::Motor& motor;
        pros::Optical& topSort;
        IntakeState state = STOPPED;
        Ring ring = NONE;

        bool sort  = false;
        int speed = 127;

        double INITIAL_POSITION = 0;
        double SEPARATION_MOVEMENT = 500;
        double SEPARATION_WAIT = 0;
        double TIME_TO_COMPLETE_SEP = 100; //45 before
};
