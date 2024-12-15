#include "subsystem/intake.hpp"
#include "pros/misc.hpp"

Intake::Intake(pros::Motor& motor): motor(motor) {};

void Intake::intakeControl(){
    while(true){
        if(this->state == STOPPED){
            this->motor.move(0);
        } else if(this->state == INTAKING){
            this->motor.move(this->speed);
        } else if(this->state == OUTTAKE){
            this->motor.move(-this->speed);
        }
        pros::delay(20);
    }
}

void Intake::set(IntakeState state, int speed){
    this->state = state;
    this->speed = speed;
}

