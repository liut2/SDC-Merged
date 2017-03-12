#include "sdcManager.hh"
/*


This class is the brain of the intersection. It implements an algorithm based on
the one described in the following paper:
Dresner, Kurt, and Peter Stone.
"A Multiagent Approach to Autonomous Intersection Management."
Journal of Artificial Intelligence Research 31 (March 2008): 591-656.
https://www.aaai.org/Papers/JAIR/Vol31/JAIR-3117.pdf.

It is used to schedule cars through an intersection that is divided into an nxn
grid (in our case 10x10). Cars get "reservations" in the intersection grid that is
represented by the time at which they will occupy the grid spots. The manager takes in
a cars request to get through the intersection and predicts its path to determine if it is clear.
If so it will allow the car to enter the intersection and record the time that the car will occupy
each grid spot on its path. If not it will reject the cars request.
*/
int sdcManager::gridSize = 10;
float sdcManager::maxTurnLeft = 6;
float sdcManager::maxTurnRight = 4;
float sdcManager::rate = 1;
float sdcManager::simTime = 0;
int sdcManager::prioritized = 0;
std::vector<std::vector<float>> sdcManager::grid = std::vector<std::vector<float>>();
std::map<int, float> sdcManager::priorityList = std::map<int, float>();

//Initialize the grid
sdcManager::sdcManager(int id){
    sdcManager::makeGrid();
}
//Sets the simulation rate
void sdcManager::setRate(float newRate){
    rate = newRate;
}
//Sets the simulation time
void sdcManager::setTime(float curSimTime){
    simTime = curSimTime;
}
//Initialize the 10x10 gridwith simTime
void sdcManager::makeGrid(){
    float tm = simTime;
    std::vector<float> columns = std::vector<float>(gridSize, tm);
    grid = std::vector<std::vector<float>> (gridSize, columns);
}
//Update a grid spot with a new time
void sdcManager::setGrid(float newtime, int x, int y){
    //If we return NaN set a spot to simTime. If there are conflicts they should be detected by neighbors anyways.
    if (newtime != newtime){
        printf("nan\n");
        newtime = simTime + 1;
    }
    grid[x][y] = newtime;
}
//Returns the time in the x,y grid spot
float sdcManager::getGrid(int x, int y){
    return grid[x][y];
}
//Prints the grid for debugging
void sdcManager::printGrid(){
    printf("*********************************\n");
    for (int row = 9; row > -1; row--){
        for (int col = 0; col < gridSize; col++){
            printf("%.1f|",grid[col][row]);
        }
        printf("\n- - - - - - - - - - - - - - - - - - - - - - - -\n");
    }
    printf("*********************************\n");
}


//Handles cars request to get through the intersection.
/***
*Implements the reservation algorithm
A car makes a request to enter the intersection. It is given the cars position, speed, direciton, and destination.
Using this information it calculates the car's route through the intersection and determines the times at which it will
reach each grid spot in its path. If all of those spots are not reserved by another car (there is a time registered in that
grid that is greater than the time the new car will reach it) then it will get a reservation and will set the grid spoits on
its path to be the time that it will leave each spot. To ensure that one car is not waiting too long, 20 seconds after a cars first
request all other cars will be denied entry and the prioritized car will go through.
***/
instruction sdcManager::reservationRequest(int carId, float x, float y, float speed, int turning, int direction, int fromDir){
    //If a car is being prioritized return immediately
    if((prioritized != 0) && (carId != prioritized)){
        //printf("prioritized: %i\n", prioritized);
        return instruction::instruction(carId, .95*speed, 0);
    }
    float tm = simTime; //update time
    if (priorityList.count(carId) > 0){
        if(priorityList[carId] != 0){
            //If a car has been waiting for more than 15 seconds
            if(simTime - priorityList[carId] > 20){
                prioritized = carId;
            }
        }
        else{
            priorityList[carId] = simTime;
        }
    }
    else{
        priorityList.insert(std::pair<int, float>(carId, simTime));
    }
    // dont accept reservation before we update rate (this is at the very beginning of the simulation)
    if(tm < .5){
        return instruction::instruction(carId, speed, 0);
    }
    // if speed is too low assume speed is 1 and give reservation based on this speed
    if (speed == 0){
        speed = 1;
    }
    float decrease = .95; //A variable determining the amount we ask cars to slow down when denied a reservation
    float driveTime = 0; //Time to reach a grid spot
    float buffer =  4.25 + speed; //A value we add to various parts of our algorithm to make it safer
    //Variables used when trying to optimize speed
    float speedIntent = 6;
    float idealSpeed = 0;
    //If we are going to give a car a reservation or not
    bool reserved = true;

    //If the car is going straight
    if(turning == 0){
      float timeWillReach = 0; //Time the car will reach a grid spot
      float timeWillLeave = 0; //Time the car will leave a grid spot
      float distance = 0; //Distance to the grid spot
      if (fromDir == 0 || fromDir == 2) { //travelling y bound (north/south)
        int xIndex = static_cast<int>(x-46); //The indicies correspond to grid spots
        for (int yIndex = 0; yIndex < gridSize; yIndex++) {
            distance = fmax(std::abs(yIndex + 45 - y)-1, 1); //All grid spots are size 1
            //P = .5at2 + vt + d
            //We buffer the distance to the grid spot
            driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer)); //Solve the quadritic equation to get drive time to the grid spot
            // If it will reach its max speed before it reaches the grid spot then the equation is different (it stops accelerating)
            if (driveTime + speed > 6){
                float t6 = 6 - speed; //time to reach top speed
                float d6 = .5*(t6*t6) + speed*t6; // distance to reach top speed
                driveTime = t6 + ((distance - buffer - d6)/6); //new time according to the new values
            }
            timeWillReach = tm + driveTime; // sim time at which we will reach the grid spot

            //We assume that the car will take up 3 spots in the grid to its left and right and check all of them
            //This acts as a static buffer on the side of the car
            for (int xWidth = 0; xWidth < 3; xWidth ++) { //xWidth is width buffer
                if (timeWillReach < sdcManager::getGrid(xIndex + xWidth, yIndex) || !reserved) {
                    reserved = false;
                    return instruction::instruction(carId, decrease*speed, 0);
                }
                else { //If we are going to get a reservation then we try to optimize the speed at which we go through the intersection
                    if((float)distance < buffer){
                        distance = buffer + distance;
                    }
                    idealSpeed = ((float)distance - buffer) / (sdcManager::getGrid(xIndex + xWidth, yIndex) - tm);
                    if (idealSpeed < 0){ //negative ideal speeds either mean there is an error or tm > than the grid spot time (the car is already out of the spot)
                        if(distance < 0){ //Since we buffer the distance it could potentially be negative (doesn't happen in practice)
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{ //SpeedIntent is our final optimized speed after going through each grid spot
                        speedIntent = fmin(speedIntent, idealSpeed);
                    }
                }
            }
        }

        if (speedIntent < 2){ //we only give cars reservations if they can get through sufficiently quickly
            return instruction::instruction(carId, decrease*speed, 0);
        }
        //If we are getting a reservation then we need to register in each grid spot on our path with the time we leave the spot
        //Previously we buffered the distance to make us reach the spot faster. This time we buffer the opposite way to increase time
        if (reserved) {
            for (int yIndex = 0; yIndex < gridSize; yIndex++) {
                distance = std::abs(yIndex + 45 - y)+1.5; //+1.5 to account for the size of the grid and a bit extra buffer
                driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
                // If it will accelerate too fast
                if (driveTime + speed > 6){
                    float t6 = 6 - speed;
                    float d6 = .5*(t6*t6) + speed*t6;
                    driveTime = t6 + ((distance + buffer - d6)/6);
                }
                timeWillLeave = tm + driveTime;
                for (int xWidth = 0; xWidth < 3; xWidth ++) { //xWidth is width buffer
                    sdcManager::setGrid(timeWillLeave, xIndex + xWidth, yIndex);
                }
            }
            prioritized = 0;
            priorityList[carId] = 0.0;
            return instruction::instruction(carId, speedIntent, 1);
         }

      }

      /*
      X BOUND
      */
      //Exactly the same as the Y bound except the indicies are different. See the y bound case for detailed comments
     else if (fromDir == 1 || fromDir == 3) {
        int yIndex = static_cast<int>(y-46);
        for (int xIndex = 0; xIndex < gridSize; xIndex++) {
            distance = fmax(std::abs(xIndex + 45 - x)-1, 1);
            driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
            // If it will accelerate too fast
            if (driveTime + speed > 6){
                float t6 = 6 - speed;
                float d6 = .5*(t6*t6) + speed*t6;
                driveTime = t6 + ((distance - buffer - d6)/6);
            }
            timeWillReach = tm + driveTime;
            for (int yWidth = 0; yWidth < 3; yWidth ++) { //xWidth is width buffer
                if (timeWillReach < sdcManager::getGrid(xIndex, yIndex + yWidth) || !reserved) {
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    if((float)distance < buffer){
                        distance = buffer + distance;
                    }
                    idealSpeed = ((float)distance - buffer) / (sdcManager::getGrid(xIndex, yIndex + yWidth) - tm);
                    if(carId == 1){
                    }
                    if (idealSpeed < 0){
                        if(distance < 0){
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        speedIntent = fmin(speedIntent, idealSpeed);
                    }
                }
            }

        }
        if(speedIntent < 2){
          return instruction::instruction(carId, speed*decrease, 0);
        }

        if (reserved) {
          for (int xIndex = 0; xIndex < gridSize; xIndex++) {
            distance = std::abs(xIndex + 45 - x)+1.5; //+2 for extra buffer
            driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
            // If it will accelerate too fast
            if (driveTime + speed > 6){
                float t6 = 6 - speed;
                float d6 = .5*(t6*t6) + speed*t6;
                driveTime = t6 + ((distance + buffer - d6)/6);
            }
            timeWillLeave = tm + driveTime;
            for (int yWidth = 0; yWidth < 3; yWidth ++) {
                sdcManager::setGrid(timeWillLeave, xIndex, yIndex + yWidth);
            }
          }
           prioritized = 0;
           priorityList[carId] = 0.0;
           return instruction::instruction(carId, speedIntent, 1);
        }
      }
      return instruction::instruction(carId, 0, 0);
    }
    //If the car is turning left go to the left method
    else if (turning == 1){
        return sdcManager::leftTurnRequest(carId, x, y, speed, turning, direction, fromDir);
    }
    //If the car is turning right go to the right method
    else if (turning == 2){
        return sdcManager::rightTurnRequest(carId, x, y, speed, turning, direction, fromDir);
    }
    //This shouldn't happen (if a car isn't going straight, left or right)
    else{
      return instruction::instruction(carId, speed*decrease, 0);
    }


}
//For left turn cars, we calculate the distance to the intersection first
//Then we assume a circular path for turning and calculate the arc length to each grid spot
//Using this we can calculate the time we should reach and leave each grid spot
instruction sdcManager::leftTurnRequest(int carId, float x, float y, float speed, int turning, int direction, int fromDir){
    //Similar set up to the straight case. See above for detailed comments
    if(speed < 1){
        speed = 1;
    }

    float tm = simTime;
    if(tm < .5){
        return instruction::instruction(carId, speed, 0);
    }
    float decrease = .95;
    bool reserved = true;
    float driveTime = 0;
    float idealSpeed = 0;
    float buffer = 4.25 + .2*speed;
    float timeWillReach = 0;
    float timeWillLeave = 0;
    float distance = 0;
    float tanValue = 0;
    float angle = 0;
    float r = 0;
    // if speed is too low assume speed is 1 and give reservation based on this speed
    if (speed == 0){
        speed = 1;
    }
    float speedIntent = maxTurnLeft; //max speed we give to cars with reservations

    //Each direction needs to check different grid spots so we split on the direction
    if (fromDir == 0) { //From N
        distance = std::abs(y - 55); //distance to the intersection from the cars current location
        //calculate the time it will take to get to the intersection
        driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
        if (driveTime + speed > 6){
            float t6 = 6 - speed;
            float d6 = .5*(t6*t6) + speed*t6;
            driveTime = t6 + ((distance - buffer - d6)/6);
        }
        //corresponds to the grid spots for a circular path
        //1. x: 0-5, y: 5-10
        //2. x: 3-7, y: 3-7
        //3. x: 5-10, y: 0-5
        //1. NC
        for (int iX = 0; iX < 5; iX++) {
            for (int iY = 5; iY < 10; iY++) {
                tanValue = (10-iY)/((float)(10 - iX)); //get the tangent from the intersection corner to the grid spot
                angle = atan(tanValue);
                if(angle < 0){
                  printf("1nc angle negative\n");
                }
                r = sqrt(pow((10-iY), 2) + pow((10 - iX),2)); //radius to the grid spot from the intersection corner
                // If it will accelerate too fast

                timeWillReach = tm + driveTime + (angle*r)/speed; //calculate the time the car will each the intersection
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                //Optimize the speed
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * .4));
                    if (idealSpeed < 0){
                        if(distance < 0){
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        speedIntent = fmin(speedIntent, idealSpeed);
                    }
                }
            }
        }
        //2. NC
        for (int iX = 3; iX < 7; iX++) {
            for (int iY = 3; iY < 7; iY++) {
                tanValue = (10-iY)/((float)(10 - iX));
                angle = atan(tanValue);
                if(angle < 0){
                  printf("nc1 angle negative\n");
                }
                r = sqrt(pow((10-iY), 2) + pow((10 - iX),2));
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * .4));
                    if (idealSpeed < 0){
                        if(distance < 0){
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        speedIntent = fmin(speedIntent, idealSpeed);
                    }
                }
            }
        }
        //3. NC
        for (int iX = 5; iX < 10; iX++) {
            for (int iY = 0; iY < 5; iY++) {
                tanValue = (10-iY)/((float)(10 - iX));
                angle = atan(tanValue);
                if(angle < 0){
                  printf("nc3 angle negative\n");
                }
                r = sqrt(pow((10-iY), 2) + pow((10 - iX),2));
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * .4));
                    if (idealSpeed < 0){
                        if(distance < 0){
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        speedIntent = fmin(speedIntent, idealSpeed);
                    }
                }
            }
        }
        if (speedIntent < 3){
            return instruction::instruction(carId, decrease*speed, 0);
        }
        //Pass through the check, can make reservation
        //Now we go through and register our times with each grid spot in our route
        if (reserved) {
            //1. NR
            for (int iX = 0; iX < 5; iX++) {
                for (int iY = 5; iY < 10; iY++) {
                    tanValue = (10-iY)/((float)(10 - iX));
                    angle = atan(tanValue);
                    if(angle < 0){
                      printf("1nr angle negative\n");
                    }
                    r = sqrt(pow((10-iY), 2) + pow((10 - iX),2));
                    float intersectionSpeed = fmin(speedIntent, speed + driveTime);
                    //time = simtime + time to get to intersection + time to grid spot
                    timeWillLeave = tm + driveTime + quadraticRoot(1, -intersectionSpeed, -(angle*r + buffer));
                    //If we will arrive using optimized speed before we should then reject
                    if( timeWillLeave < sdcManager::getGrid(iX,iY)){
                        return instruction::instruction(carId, speed*decrease, 0);
                    }
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            //2. NR
            for (int iX = 3; iX < 7; iX++) {
                for (int iY = 3; iY < 7; iY++) {
                    tanValue = (10-iY)/((float)(10-iX));
                    angle = atan(tanValue);
                    if(angle < 0){
                      printf("2nr angle negative\n");
                    }
                    r = sqrt(pow((10-iY), 2) + pow((10 - iX),2));
                    float intersectionSpeed = fmin(speedIntent, speed + driveTime);
                    timeWillLeave = tm + driveTime + quadraticRoot(1, -intersectionSpeed, -(angle*r + buffer));
                    //If we will arrive using optimized speed before we should then reject
                    if( timeWillLeave < sdcManager::getGrid(iX,iY)){
                        return instruction::instruction(carId, speed*decrease, 0);
                    }
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            //3. NR
            for (int iX = 5; iX < 10; iX++) {
                for (int iY = 0; iY < 5; iY++) {
                    tanValue = (10-iY)/((float)(10 - iX));
                    angle = atan(tanValue);
                    if(angle < 0){
                      printf("3nr angle negative\n");
                    }
                    r = sqrt(pow((10-iY), 2) + pow((10 - iX),2));
                    float intersectionSpeed = fmin(speedIntent, speed + driveTime);
                    //time + time to intersection + time in intersection
                    timeWillLeave = tm + driveTime + quadraticRoot(1, -intersectionSpeed, -(angle*r + buffer));
                    //If we will arrive using optimized speed before we should then reject
                    if( timeWillLeave < sdcManager::getGrid(iX,iY)){
                        return instruction::instruction(carId, speed*decrease, 0);
                    }
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            prioritized = 0;
            priorityList[carId] = 0.0;
            return instruction::instruction(carId, speedIntent, 1);
        }
    }

    /*
     SOUTH CASE
     */
     //All of the cases are the same as the north case but with different indicies. See above for detailed comments
    else if ( fromDir == 2){ //From S
        distance = std::abs(45 - y);
        driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
        // If it will accelerate too fast
        if (driveTime + speed > 6){
            float t6 = 6 - speed;
            float d6 = .5*(t6*t6) + speed*t6;
            driveTime = t6 + ((distance - buffer - d6)/6);
        }
        //1. x: 5-10, y: 0-5
        //2. x: 3-7, y: 3-7
        //3. x: 0-5, y: 5-10
        //1.SC
        for (int iX = 5; iX < 10; iX++) {
            for (int iY = 0; iY < 5; iY++) {
                tanValue = iY/((float)iX);
                angle = atan(tanValue);
                r = sqrt(pow(iX, 2) + pow(iY,2));
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * .4));
                    if (idealSpeed < 0){
                        if(distance < 0){
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        speedIntent = fmin(speedIntent, idealSpeed);
                    }
                }
            }
        }
        //2.SC
        for (int iX = 3; iX < 7; iX++) {
            for (int iY = 3; iY < 7; iY++) {
                tanValue = iY/((float)iX);
                angle = atan(tanValue);
                r = sqrt(pow(iX, 2) + pow(iY,2));
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * .4));
                    if (idealSpeed < 0){
                        if(distance < 0){
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        speedIntent = fmin(speedIntent, idealSpeed);
                    }
                }
            }
        }
        //3.SC
        for (int iX = 0; iX < 5; iX++) {
            for (int iY = 5; iY < 10; iY++) {
                tanValue = iY/((float)iX);
                angle = atan(tanValue);

                r = sqrt(pow(iX, 2) + pow(iY,2));
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * .4));
                    if (idealSpeed < 0){
                        if(distance < 0){
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        speedIntent = fmin(speedIntent, idealSpeed);
                    }
                }
            }
        }
        if (speedIntent < 3){
            return instruction::instruction(carId, decrease*speed, 0);
        }
        //Pass through the check, can make reservation
        if (reserved) {
            //1.SR
            for (int iX = 5; iX < 10; iX++) {
                for (int iY = 0; iY < 5; iY++) {
                    tanValue = iY/((float)iX);
                    angle = atan(tanValue);
                    r = sqrt(pow(iX, 2) + pow(iY,2));
                    float intersectionSpeed = fmin(speedIntent, speed + driveTime);
                    timeWillLeave = tm + driveTime + quadraticRoot(1, -intersectionSpeed, -(angle*r + buffer));
                    if( timeWillLeave < sdcManager::getGrid(iX,iY)){
                        return instruction::instruction(carId, speed*decrease, 0);
                    }
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            //2.SR
            for (int iX = 3; iX < 7; iX++) {
                for (int iY = 3; iY < 7; iY++) {
                    tanValue = iY/((float)iX);
                    angle = atan(tanValue);
                    r = sqrt(pow(iX, 2) + pow(iY,2));
                    float intersectionSpeed = fmin(speedIntent, speed + driveTime);
                    timeWillLeave = tm + driveTime + quadraticRoot(1, -intersectionSpeed, -(angle*r + buffer));
                    if( timeWillLeave < sdcManager::getGrid(iX,iY)){
                        return instruction::instruction(carId, speed*decrease, 0);
                    }
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            //3.SR
            for (int iX = 0; iX < 5; iX++) {
                for (int iY = 5; iY < 10; iY++) {
                    tanValue = iY/((float)iX);
                    angle = atan(tanValue);
                    r = sqrt(pow(iX, 2) + pow(iY,2));
                    float intersectionSpeed = fmin(speedIntent, speed + driveTime);
                    timeWillLeave = tm + driveTime + quadraticRoot(1, -intersectionSpeed, -(angle*r + buffer));
                    if( timeWillLeave < sdcManager::getGrid(iX,iY)){
                        return instruction::instruction(carId, speed*decrease, 0);
                    }
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            prioritized = 0;
            priorityList[carId] = 0.0;
            return instruction::instruction(carId, speedIntent, 1);
        }
    }

    /*
     EAST CASE
     */

    else if ( fromDir == 1){ //From E
        distance = std::abs(x - 55);
        driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
        // If it will accelerate too fast
        if (driveTime + speed > 6){
            float t6 = 6 - speed;
            float d6 = .5*(t6*t6) + speed*t6;
            driveTime = t6 + ((distance - buffer - d6)/6);
        }
        //1. x: 5-10, y: 5-10
        //2. x: 3-7, y: 3-7
        //3. x: 0-5, y: 0-5
        //1. EC
        for (int iX = 5; iX < 10; iX++) {
            for (int iY = 5; iY < 10; iY++) {
                tanValue = (10-iX)/((float)iY);
                angle = atan(tanValue);
                if(angle < 0){
                  printf("ec angle negative\n");
                }
                r = sqrt(pow((10-iX), 2) + pow(iY,2));
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * .4));
                    if (idealSpeed < 0){
                        if(distance < 0){
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        speedIntent = fmin(speedIntent, idealSpeed);
                    }
                }
            }
        }
        //2. EC
        for (int iX = 3; iX < 7; iX++) {
            for (int iY = 3; iY < 7; iY++) {
                tanValue = (10-iX)/((float)iY);
                angle = atan(tanValue);
                if(angle < 0){
                  printf("ec angle negative\n");
                }
                r = sqrt(pow((10-iX), 2) + pow(iY,2));
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * .4));
                    if (idealSpeed < 0){
                        if(distance < 0){
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        speedIntent = fmin(speedIntent, idealSpeed);
                    }
                }
            }
        }
        //3. EC
        for (int iX = 0; iX < 5; iX++) {
            for (int iY = 0; iY < 5; iY++) {
                tanValue = (10-iX)/((float)iY);
                angle = atan(tanValue);
                r = sqrt(pow((10-iX), 2) + pow(iY,2));
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * .4));
                    if (idealSpeed < 0){
                        if(distance < 0){
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        speedIntent = fmin(speedIntent, idealSpeed);
                    }
                }
            }
        }
        if (speedIntent < 3){
            return instruction::instruction(carId, decrease*speed, 0);
        }
        //Pass through the check, can make reservation
        if (reserved) {
            //1.ER
            for (int iX = 5; iX < 10; iX++) {
                for (int iY = 5; iY < 10; iY++) {
                    tanValue = (10-iX)/((float)iY);
                    angle = atan(tanValue);
                    r = sqrt(pow((10-iX), 2) + pow(iY,2));
                    float intersectionSpeed = fmin(speedIntent, speed + driveTime);
                    timeWillLeave = tm + driveTime + quadraticRoot(1, -intersectionSpeed, -(angle*r + buffer));
                    if( timeWillLeave < sdcManager::getGrid(iX,iY)){
                        return instruction::instruction(carId, speed*decrease, 0);
                    }
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            //2.ER
            for (int iX = 3; iX < 7; iX++) {
                for (int iY = 3; iY < 7; iY++) {
                    tanValue = (10-iX)/((float)iY);
                    angle = atan(tanValue);
                    r = sqrt(pow((10-iX), 2) + pow(iY,2));
                    float intersectionSpeed = fmin(speedIntent, speed + driveTime);
                    timeWillLeave = tm + driveTime + quadraticRoot(1, -intersectionSpeed, -(angle*r + buffer));
                    if( timeWillLeave < sdcManager::getGrid(iX,iY)){
                        return instruction::instruction(carId, speed*decrease, 0);
                    }
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            //3.ER
            for (int iX = 0; iX < 5; iX++) {
                for (int iY = 0; iY < 5; iY++) {
                    tanValue = (10-iX)/((float)iY);
                    angle = atan(tanValue);
                    r = sqrt(pow((10-iX), 2) + pow(iY,2));
                    float intersectionSpeed = fmin(speedIntent, speed + driveTime);
                    timeWillLeave = tm + driveTime + quadraticRoot(1, -intersectionSpeed, -(angle*r + buffer));
                    if( timeWillLeave < sdcManager::getGrid(iX,iY)){
                        return instruction::instruction(carId, speed*decrease, 0);
                    }
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            prioritized = 0;
            priorityList[carId] = 0.0;
            return instruction::instruction(carId, speedIntent, 1);
        }
    }

    /*
     WEST CASE
     */

    else if ( fromDir == 3){ //From W
        distance = std::abs(45 - x);
        driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
        // If it will accelerate too fast
        if (driveTime + speed > 6){
            float t6 = 6 - speed;
            float d6 = .5*(t6*t6) + speed*t6;
            driveTime = t6 + ((distance - buffer - d6)/6);
        }
        //1. x: 0-5, y: 0-5
        //2. x: 3-7, y: 3-7
        //3. x: 5-10, y: 5-10
        //1. WC
        for (int iX = 0; iX < 5; iX++) {
            for (int iY = 0; iY < 5; iY++) {
                tanValue = (iX)/((float)(10-iY));
                angle = atan(tanValue);
                r = sqrt(pow((iX), 2) + pow((10-iY),2));
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * .4));
                    if (idealSpeed < 0){
                        if(distance < 0){
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        speedIntent = fmin(speedIntent, idealSpeed);
                    }
                }
            }
        }
        //2. WC
        for (int iX = 3; iX < 7; iX++) {
            for (int iY = 3; iY < 7; iY++) {
                tanValue = (iX)/((float)(10-iY));
                angle = atan(tanValue);
                r = sqrt(pow((iX), 2) + pow((10-iY),2));
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * .4));
                    if (idealSpeed < 0){
                        if(distance < 0){
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        speedIntent = fmin(speedIntent, idealSpeed);
                    }
                }
            }
        }
        //3. WC
        for (int iX = 5; iX < 10; iX++) {
            for (int iY = 5; iY < 10; iY++) {
                tanValue = (iX)/((float)(10-iY));
                angle = atan(tanValue);
                r = sqrt(pow((iX), 2) + pow((10-iY),2));
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * .4));
                    if (idealSpeed < 0){
                        if(distance < 0){
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        speedIntent = fmin(speedIntent, idealSpeed);
                    }
                }
            }
        }
        if (speedIntent < 3){
            return instruction::instruction(carId, decrease*speed, 0);
        }
        //Pass through the check, can make reservation
        if (reserved) {
            //1.WR
            for (int iX = 0; iX < 5; iX++) {
                for (int iY = 0; iY < 5; iY++) {
                    tanValue = (iX)/((float)(10-iY));
                    angle = atan(tanValue);
                    if(angle < 0){
                      printf("1wr angle negative\n");
                    }
                    r = sqrt(pow((iX), 2) + pow((10-iY),2));
                    float intersectionSpeed = fmin(speedIntent, speed + driveTime);
                    timeWillLeave = tm + driveTime + quadraticRoot(1, -intersectionSpeed, -(angle*r + buffer));
                    if( timeWillLeave < sdcManager::getGrid(iX,iY)){
                        return instruction::instruction(carId, speed*decrease, 0);
                    }
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            //2.WR
            for (int iX = 3; iX < 7; iX++) {
                for (int iY = 3; iY < 7; iY++) {
                    tanValue = (iX)/((float)(10-iY));
                    angle = atan(tanValue);
                    if(angle < 0){
                      printf("2wr angle negative\n");
                    }
                    r = sqrt(pow((iX), 2) + pow((10-iY),2));
                    float intersectionSpeed = fmin(speedIntent, speed + driveTime);
                    timeWillLeave = tm + driveTime + quadraticRoot(1, -intersectionSpeed, -(angle*r + buffer));
                    if( timeWillLeave < sdcManager::getGrid(iX,iY)){
                        return instruction::instruction(carId, speed*decrease, 0);
                    }
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            //3.WR
            for (int iX = 5; iX < 10; iX++) {
                for (int iY = 5; iY < 10; iY++) {
                    tanValue = (iX)/((float)(10-iY));
                    angle = atan(tanValue);
                    if(angle < 0){
                      printf("3 wr angle negative\n");
                    }
                    r = sqrt(pow((iX), 2) + pow((10-iY),2));
                    float intersectionSpeed = fmin(speedIntent, speed + driveTime);
                    timeWillLeave = tm + driveTime + quadraticRoot(1, -intersectionSpeed, -(angle*r + buffer));
                    if( timeWillLeave < sdcManager::getGrid(iX,iY)){
                        return instruction::instruction(carId, speed*decrease, 0);
                    }
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            prioritized = 0;
            priorityList[carId] = 0.0;
            return instruction::instruction(carId, speedIntent, 1);
        }
    }
    return instruction::instruction(carId, speed*.2, 0);

}

instruction sdcManager::rightTurnRequest(int carId, float x, float y, float speed, int turning, int direction, int fromDir){
  //Similar setup to straight and left turning cases. see above for detailed comments
    if(speed < 1){
        speed = 1;
    }

    float tm = simTime;
    if(tm < .5){
        return instruction::instruction(carId, speed, 0);
    }
    float decrease = .95;
    bool reserved = true;
    float driveTime = 0;
    float idealSpeed = 0;
    float buffer = 4.25 + .2*speed;
    float timeWillReach = 0;
    float timeWillLeave = 0;
    float distance = 0;
    float tanValue = 0;
    float angle = 0;
    float r = 0;
    // if speed is too low assume speed is 1 and give reservation based on this speed
    if (speed == 0){
      speed = 1;
    }
    float speedIntent = maxTurnRight; //max speed we give to cars with reservations

    //The grid spots on our path depend on the direction we come from so we split on direction
    //We assume a circular turning path. We start by calculating the distance and time to reach the entrance of the intersection
    //Then we calculate the arc length to each grid spot in the path and calculate the time to reach it.
    if (fromDir == 0) { //From N
        distance = std::abs(y - 55);
        driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
        // If it will accelerate too fast
        if (driveTime + speed > 6){
            float t6 = 6 - speed;
            float d6 = .5*(t6*t6) + speed*t6;
            driveTime = t6 + ((distance - buffer - d6)/6);
        }
        //Checking if we can make reservation
        for (int iX = 0; iX < 4; iX++) { //loop x
            for (int iY = 7; iY < 10; iY++) { //loop y
                tanValue = (10-iY)/((float)iX);
                if(iX == 0){
                    tanValue = 1;
                }
                angle = atan(tanValue);
                r = sqrt(pow((10-iY), 2) + pow(iX,2));
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX, iY) - tm + (buffer * .4));
                    if (idealSpeed < 0){
                        if(distance < 0){
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        speedIntent = fmin(speedIntent, idealSpeed);
                    }
                }
            }
        }
        if (speedIntent < 2){
          return instruction::instruction(carId, decrease*speed, 0);
        }
        //Pass through the check, can make reservation
        if (reserved) {
            for (int iX = 0; iX < 4; iX++) { //loop x
                for (int iY = 7; iY < 10; iY++) { //loop y
                    tanValue = (10-iY)/((float)iX);
                    if(iX == 0){
                        tanValue = 1;
                    }
                    angle = atan(tanValue);
                    r = sqrt(pow((10-iY), 2) + pow(iX,2));
                    float intersectionSpeed = fmin(speedIntent, speed + driveTime);
                    timeWillLeave = tm + driveTime + quadraticRoot(1, -intersectionSpeed, -(angle*r + buffer));
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            prioritized = 0;
            priorityList[carId] = 0.0;
            return instruction::instruction(carId, speedIntent, 1);
        }
    }

    /*
    SOUTH CASE
    */

    else if ( fromDir == 2){ //From S
        distance = std::abs(45 - y);
        driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
        // If it will accelerate too fast
        if (driveTime + speed > 6){
            float t6 = 6 - speed;
            float d6 = .5*(t6*t6) + speed*t6;
            driveTime = t6 + ((distance - buffer - d6)/6);
        }
        //Checking if we can make reservation
        for (int iX = 6; iX < 10; iX++) { //loop x
            for (int iY = 0; iY < 3; iY++) { //loop y
                tanValue = ((float)iY)/(10 - iX);
                angle = atan(tanValue);
                r = sqrt(pow(iY, 2) + pow((10 - iX),2));
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                  reserved = false;
                  return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX, iY) - tm + (buffer * .4));
                    if (idealSpeed < 0){
                        if(distance < 0){
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        speedIntent = fmin(speedIntent, idealSpeed);
                    }
                }
            }
        }
        if (speedIntent < 2){
          return instruction::instruction(carId, decrease*speed, 0);
        }
        //Pass through the check, can make reservation
        if (reserved) {
            for (int iX = 6; iX < 10; iX++) { //loop x
                for (int iY = 0; iY < 3; iY++) { //loop y
                    tanValue = ((float)iY)/(10-iX);
                    angle = atan(tanValue);
                    r = sqrt(pow(iY, 2) + pow((10 - iX),2));
                    float intersectionSpeed = fmin(speedIntent, speed + driveTime);
                    timeWillLeave = tm + driveTime + quadraticRoot(1, -intersectionSpeed, -(angle*r + buffer));
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            prioritized = 0;
            priorityList[carId] = 0.0;
            return instruction::instruction(carId, speedIntent, 1);
        }
    }

    /*
    EAST CASE
    */

    else if ( fromDir == 1){ //From E
        distance = std::abs(x - 55);
        driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
        // If it will accelerate too fast
        if (driveTime + speed > 6){
            float t6 = 6 - speed;
            float d6 = .5*(t6*t6) + speed*t6;
            driveTime = t6 + ((distance - buffer - d6)/6);
        }
        //Checking if we can make reservation
        for (int iX = 7; iX < 10; iX++) { //loop x
            for (int iY = 6; iY < 10; iY++) { //loop y
                tanValue = (10-iX)/((float)(10 - iY));
                angle = atan(tanValue);
                r = sqrt(pow((10 - iY), 2) + pow((10 - iX),2));
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                  reserved = false;
                  return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX, iY) - tm + (buffer * .4));
                    if (idealSpeed < 0){
                        if(distance < 0){
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        speedIntent = fmin(speedIntent, idealSpeed);
                    }
                }
            }
        }
        if (speedIntent < 2){
          return instruction::instruction(carId, decrease*speed, 0);
        }
        //Pass through the check, can make reservation
        if (reserved) {
            for (int iX = 7; iX < 10; iX++) { //loop x
                for (int iY = 6; iY < 10; iY++) { //loop y
                    tanValue = (10-iX)/((float)(10 - iY));
                    angle = atan(tanValue);
                    r = sqrt(pow((10 - iY), 2) + pow((10 - iX),2));
                    float intersectionSpeed = fmin(speedIntent, speed + driveTime);
                    timeWillLeave = tm + driveTime + quadraticRoot(1, -intersectionSpeed, -(angle*r + buffer));
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            prioritized = 0;
            priorityList[carId] = 0.0;
            return instruction::instruction(carId, speedIntent, 1);
        }
    }

    /*
    WEST CASE
    */

    else if ( fromDir == 3){ //From W
        distance = std::abs(45 - x);
        driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
        // If it will accelerate too fast
        if (driveTime + speed > 6){
            float t6 = 6 - speed;
            float d6 = .5*(t6*t6) + speed*t6;
            driveTime = t6 + ((distance - buffer - d6)/6);
        }
        //Checking if we can make reservation
        for (int iX = 0; iX < 3; iX++) { //loop x
            for (int iY = 0; iY < 4; iY++) { //loop y
                tanValue = iX/((float)iY);
                angle = atan(tanValue);
                r = sqrt(pow(iY, 2) + pow(iX,2));
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                  reserved = false;
                  return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX, iY) - tm + (buffer * .4));
                    if (idealSpeed < 0){
                        if(distance < 0){
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        speedIntent = fmin(speedIntent, idealSpeed);
                    }
                }
            }
        }
        if (speedIntent < 2){
          return instruction::instruction(carId, decrease*speed, 0);
        }
        //Pass through the check, can make reservation
        if (reserved) {
            for (int iX = 0; iX < 3; iX++) { //loop x
                for (int iY = 0; iY < 4; iY++) { //loop y
                    tanValue = iX/((float)iY);
                    angle = atan(tanValue);
                    r = sqrt(pow(iY, 2) + pow(iX,2));
                    float leaveSpeed = fmin(speed, maxTurnRight);
                    float intersectionSpeed = fmin(speedIntent, speed + driveTime);
                    timeWillLeave = tm + driveTime + quadraticRoot(1, -intersectionSpeed, -(angle*r + buffer));
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            prioritized = 0;
            priorityList[carId] = 0.0;
            return instruction::instruction(carId, speedIntent, 1);
        }
    }
    return instruction::instruction(carId, speed*.2, 0);
}

//Performs the quadritic formula
float sdcManager::quadraticRoot(float a, float b, float c){
    float D = sqrt((b*b) - (4*a*c));
    if (!a)
    {
        printf("divide by zero\n");
        return 0;
    }
    float root = ( -b + D)/(2*a);
    if (root != root){
        return 0;
    }
    return root;//function succeded only return larger root
}
