#include "instruction.hh"

/**
Class used by intersection managers to communicate with cars
**/

instruction::instruction(int id, float inputSpeed, bool inputHasReservation){
    this->id = id;
    this->speed = inputSpeed;
    this->hasReservation = inputHasReservation;

}

void instruction::printid(){
    printf("id is: ");
    printf("%d", id);
}

float instruction::getSpeed() {
    return this->speed;
}

bool instruction::getHasReservation() {
    return this->hasReservation;
}
