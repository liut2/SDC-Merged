#ifndef INSTRUCTION_HH_
#define INSTRUCTION_HH_

#include <iostream>

class instruction{
    public: 
        instruction(int id, int inputSpeed, bool inputHasReservation);
        float getSpeed();
    bool getHasReservation();
        int id = 0;
        void printid();
        float speed = 0;
        bool hasReservation = 0;
    
    
};
//extern manager manager;
#endif