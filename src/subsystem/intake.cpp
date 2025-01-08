#include "subsystem/intake.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

Intake::Intake(pros::Motor& motor, pros::Optical& topSort, Arm& arm): motor(motor) , topSort(topSort) , arm(arm){};


void Intake::setSeparation(Ring ring){
    this->ring = ring;
}

void Intake::checkForSort(){
    if (ring == Ring::NONE){
       sort = false; 
    }
    else if (ring == Ring::BLUE){
        if (topSort.get_hue()>200 && topSort.get_hue()<=270){
            sort = true;
            INITIAL_POSITION = motor.get_position();
        }
    }
    else if (ring == Ring::RED){
        if (topSort.get_hue()<30 && topSort.get_hue()>=0) {
            sort = true;
            INITIAL_POSITION = motor.get_position();
        }
    }
}

void Intake::intakeControl(){
    while(true){

        if (!sort){
            checkForSort();
        }
        // bool COMPLETED_MOVEMENT = (INITIAL_POSITION+SEPARATION_MOVEMENT) <= motor.get_position();

        // bool COMPLETED_SEPARATION_WAIT = SEPARATION_WAIT >= TIME_TO_COMPLETE_SEP;

        // if (sort && !COMPLETED_MOVEMENT) {
        //     set(IntakeState::INTAKING,90);
        //     this->motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        // }
        // else if (sort && COMPLETED_MOVEMENT && !COMPLETED_SEPARATION_WAIT){
        //     this->state = STOPPED;
        //     SEPARATION_WAIT++;
        // }
        // else if (sort && COMPLETED_MOVEMENT && COMPLETED_SEPARATION_WAIT) {
        //     set(IntakeState::INTAKING,127);
        //     sort = !sort;
        //     SEPARATION_WAIT = 0;
        //     this->motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        // }
        // if(sort){ arm.separateRing(5,true); sort = !sort; }
        
        


        if(this->state == STOPPED){
            this->motor.brake();
        } else if(this->state == INTAKING){
            this->motor.move(this->speed);
        } else if(this->state == OUTTAKE){
            this->motor.move(-this->speed);
        }
        pros::delay(10);
    }
}

void Intake::set(IntakeState state, int speed){
    this->state = state;
    this->speed = speed;
}

