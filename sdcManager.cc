#include "sdcManager.hh"

int sdcManager::carAmt = 0;
int sdcManager::currentDir = 0;
int sdcManager::currentTurn = 0;
bool sdcManager::nStop = 0;
bool sdcManager::eStop = 0;
bool sdcManager::sStop = 0;
bool sdcManager::wStop = 0;
int sdcManager::gridSize = 10;
float sdcManager::maxTurnLeft = 6;
float sdcManager::maxTurnRight = 4;
float sdcManager::rate = 1;
std::chrono::milliseconds sdcManager::msStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
time_t sdcManager::startTime = time(0);
std::vector<int> sdcManager::carList = std::vector<int>();
std::vector<std::vector<float>> sdcManager::grid = std::vector<std::vector<float>>();
std::vector<int> sdcManager::carNorthQueue = std::vector<int>();
std::vector<int> sdcManager::carEastQueue = std::vector<int>();
std::vector<int> sdcManager::carSouthQueue = std::vector<int>();
std::vector<int> sdcManager::carWestQueue = std::vector<int>();
std::vector<gazebo::sdcSensorData> sdcManager::sensorDataList = std::vector<gazebo::sdcSensorData>();
std::map<int, gazebo::sdcSensorData> sdcManager::sensorManager = std::map<int, gazebo::sdcSensorData>();

sdcManager::sdcManager(int id){
    //printf("created manager\n");
    printf("%d\n", id);
    sdcManager::makeGrid();
    //sdcManager::setGrid(1, 2, 3);
    //printf("Spot 2,3: %i\n", sdcManager::getGrid(2,3));
    //printf("Spot 3,3: %i\n", sdcManager::getGrid(3,3));
}

void sdcManager:: printid(){
    //printf("id is: ");
    printf("%d", id);
}

void sdcManager::setRate(float newRate){
    rate = newRate;
    //printf("------rate: %f------\n", rate);
}

void sdcManager::makeGrid(){
    //time_t tm = difftime(time(0), startTime);
    std::chrono::milliseconds currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    int diffTime = currentTime.count() - msStartTime.count();
    float tm = diffTime * rate / float(1000);
    printf("milliTime: %f\n", tm);
    std::vector<float> columns = std::vector<float>(gridSize, tm);
    grid = std::vector<std::vector<float>> (gridSize, columns);
    //    for(int x = 0; x < gridSize; x++) {
    //        for (int y = 0; y < gridSize; y++) {
    //            setGrid(0, x, y);
    //        }
    //    }
}

void sdcManager::setGrid(float filled, int x, int y){
    //std::vector<std::vector<int>> grid (
    //std::vector<int> temp = std::vector<int>();
    //temp.at(x) = filled;
    //grid.at(y) = temp;
    grid[x][y] = filled;
}

float sdcManager::getGrid(int x, int y){
    //printf("in get grid\n");
    return grid[x][y];
}

void sdcManager::registerCar(int carId, int turning, int direction) {
    carAmt++;
    currentDir = direction;
    currentTurn = turning;
    //printf("registered, and now carAmt is: %i\n", carAmt);
    fflush(stdout);
    carList.push_back(carId);
    //printf("registered car Id is: %i\n", carId);
    fflush(stdout);

}
void sdcManager::laneStopRequest(int fromDir){
    switch (fromDir) {
        case 0:
            //printf("nstop\n");
            nStop = true;
            break;
        case 1:
            //printf("estop\n");
            eStop = true;
            break;
        case 2:
            //printf("sstop\n");
            sStop = true;
            break;
        case 3:
            //printf("wstop\n");
            wStop = true;
            break;
    }
}

instruction sdcManager::reservationRequest(int carId, float x, float y, float speed, int turning, int direction, int fromDir){
    std::chrono::milliseconds currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    int diffTime = currentTime.count() - msStartTime.count();
    float tm = diffTime * rate / float(1000);
    // dont accept reservation before we update rate
    if(diffTime/float(1000) < .5){
        return instruction::instruction(carId, speed, 0);
    }
    //printf("tm: %f\n", tm);
    // if speed is too low assume speed is 1 and give reservation based on this speed
    if (speed == 0){
        speed = 1;
    }
    float decrease = .95;
    float driveTime = 0;
    float buffer = fmax(4.25, 4.25 + speed);
    //printf("buffer: %f\n", buffer);
    float speedIntent = 6; //max speed we give to cars with reservations
    float idealSpeed = 0;
    bool reserved = true;
    if(turning == 0){
      float timeWillReach = 0;
      float timeWillLeave = 0;
      float distance = 0;
      if (fromDir == 0 || fromDir == 2) { //travelling y bound
        int xIndex = static_cast<int>(x-46);
        //printf("carId is y bound: %d\n", carId);
        for (int yIndex = 0; yIndex < gridSize; yIndex++) {
            distance = std::abs(yIndex + 45 - y);
            driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
            // If it will accelerate too fast
            //printf("carId: %i original reachdriveTime: %f\n", carId, driveTime);
            if (driveTime + speed > 6){
                float t6 = 6 - speed;
                float d6 = .5*(t6*t6) + speed*t6;
                driveTime = t6 + ((distance - buffer - d6)/6);
            }
            timeWillReach = tm + driveTime;
            for (int xWidth = 0; xWidth < 3; xWidth ++) { //xWidth is width buffer
                if (timeWillReach < sdcManager::getGrid(xIndex + xWidth, yIndex) || !reserved) {
                    reserved = false;
                    //printf("car rejected: %d\n", carId);
                    return instruction::instruction(carId, decrease*speed, 0);
                }
                else {
                    idealSpeed = fmin(speedIntent, ((float)distance - buffer) / (sdcManager::getGrid(xIndex + xWidth, yIndex) - tm));
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
        if (reserved) {
            printf("speed: %f for car: carId: %i\n", speed, carId);
            for (int yIndex = 0; yIndex < gridSize; yIndex++) {
                distance = std::abs(yIndex + 45 - y);
                //timeWillReach = tm + (distance - (speed * 1.5)) / speed;
                driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
                // If it will accelerate too fast
                //printf("carId: %i original driveTime: %f\n", carId, driveTime);
                if (driveTime + speed > 6){
                    float t6 = 6 - speed;
                    float d6 = .5*(t6*t6) + speed*t6;
                    driveTime = t6 + ((distance + buffer - d6)/6);

                    //driveTime = (pow(speed,2) - (10*speed) + (2*(distance + buffer) + 36))/12;
                    //printf("driveTime: %f\n", driveTime);
                }
                timeWillLeave = tm + driveTime;
                for (int xWidth = 0; xWidth < 3; xWidth ++) { //xWidth is width buffer
                    sdcManager::setGrid(timeWillLeave, xIndex + xWidth, yIndex);
                }
            }
            printf("carId got reserved: %d\n", carId);
            //printf("carId intend: %d, %f\n", carId, speedIntent);
            printf ("speedIntent: %f",speedIntent);
            printf("timeWillLeave: %f\n", timeWillLeave);
            printf("tm: %f, driveTime: %f\n", tm, driveTime);
            return instruction::instruction(carId, speedIntent, 1);
         }

      }

      /*
      X BOUND
      */

     else if (fromDir == 1 || fromDir == 3) {
      //  printf("carId is x bound: %d\n", carId);
        int yIndex = static_cast<int>(y-46);
        for (int xIndex = 0; xIndex < gridSize; xIndex++) {
            distance = std::abs(xIndex + 45 - x);
            driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
            // If it will accelerate too fast
            //printf("carId: %i original reachdriveTime: %f\n", carId, driveTime);
            if (driveTime + speed > 6){
                float t6 = 6 - speed;
                float d6 = .5*(t6*t6) + speed*t6;
                driveTime = t6 + ((distance - buffer - d6)/6);

                //driveTime = (pow(speed,2) - (10*speed) + (2*(distance + buffer) + 36))/12;
            //    printf("reachdriveTime: %f\n", driveTime);
            }
            timeWillReach = tm + driveTime;
          //timeWillLeave = tm + ((float)distance - buffer) / speed;

          //printf("car old leave diff: %f, %f\n", sdcManager::getGrid(xIndex, yIndex), timeWillReach);
            for (int yWidth = 0; yWidth < 3; yWidth ++) { //xWidth is width buffer
                if (timeWillReach < sdcManager::getGrid(xIndex, yIndex + yWidth) || !reserved) {
                    reserved = false;
                    //printf("car rejected: %d\n", carId);
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = fmin(speedIntent, ((float)distance - buffer) / (sdcManager::getGrid(xIndex, yIndex + yWidth) - tm));
                    if(carId == 1){
                        //printf("idealSpeed: %f tm: %ld\n", idealSpeed, tm);
                    }
                    if (idealSpeed < 0){
                        if(distance < 0){
                            //printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                        else{
                            if(carId == 1){
                                //printf("carId: %i, tm: %ld, gridtime: %f\n",carId, tm, sdcManager::getGrid(xIndex, yWidth + yIndex));
                            }
                        }
                    /*
                    else{
                        printf("tm: %ld, gridtime: %f\n", tm, sdcManager::getGrid(xIndex + xWidth, yIndex));
                        speedIntent = fmin(speedIntent, 6);
                        printf("speedIntent: %f\n", speedIntent);
                    }
                    */
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
            distance = std::abs(xIndex + 45 - x);
            //timeWillReach = tm + (distance - buffer) / speed;
            driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
            // If it will accelerate too fast
            //printf("carId: %i original driveTime: %f\n", carId, driveTime);
            if (driveTime + speed > 6){
                float t6 = 6 - speed;
                float d6 = .5*(t6*t6) + speed*t6;
                driveTime = t6 + ((distance + buffer - d6)/6);
                //float kietdriveTime = (pow(speed,2) - (10*speed) + (2*(distance + buffer) + 36))/12;
                //printf("driveTime: %f\n", driveTime);
            }
            timeWillLeave = tm + driveTime;
            for (int yWidth = 0; yWidth < 3; yWidth ++) {
                sdcManager::setGrid(timeWillLeave, xIndex, yIndex + yWidth);
            }
          }
          // if(speed < 1){
          //   speed = 10;
          // }
         // printf("carId got reserved: %d, %f\n", carId, speedIntent);
           printf ("speedIntent: %f",speedIntent);
           printf("timeWillLeave: %f\n", timeWillLeave);
           printf("tm: %f, driveTime: %f\n", tm, driveTime);
           return instruction::instruction(carId, speedIntent, 1);
        }
      }
      return instruction::instruction(carId, 0, 0);
    }
    //left?
    else if (turning == 1){
      //  printf("maxTurnLeft: %f\n", sdcManager::maxTurnLeft);
        return sdcManager::leftTurnRequest(carId, x, y, speed, turning, direction, fromDir);
    }
    else if (turning == 2){
        //printf("turning right\n");

        return sdcManager::rightTurnRequest(carId, x, y, speed, turning, direction, fromDir);
    }
    else{
      return instruction::instruction(carId, speed*decrease, 0);
    }


}

instruction sdcManager::leftTurnRequest(int carId, float x, float y, float speed, int turning, int direction, int fromDir){
    if(speed < 1){
        speed = 1;
    }
    std::chrono::milliseconds currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    int diffTime = currentTime.count() - msStartTime.count();
    float tm = diffTime *rate / float(1000);
    float decrease = .95;
    bool reserved = true;
    float driveTime = 0;
    float idealSpeed = 0;
    float buffer = fmax(4.25, 4.25 + speed);
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

    if (fromDir == 0) { //From N
        distance = std::abs(y - 55);
        //1. x: 0-5, y: 5-10
        //2. x: 3-7, y: 3-7
        //3. x: 5-10, y: 0-5
        //1. NC
        for (int iX = 0; iX < 5; iX++) {
            for (int iY = 5; iY < 10; iY++) {
                tanValue = (10-iY)/((float)(10 - iX));
                angle = atan(tanValue);
                if(angle < 0){
                  printf("1nc angle negative\n");
                }
                r = sqrt(pow((10-iY), 2) + pow((10 - iX),2));
                driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
                // If it will accelerate too fast
                if (driveTime + speed > 6){
                    float t6 = 6 - speed;
                    float d6 = .5*(t6*t6) + speed*t6;
                    driveTime = t6 + ((distance - buffer - d6)/6);
                }
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * (1/rate) * .26));
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
                driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
                // If it will accelerate too fast
                if (driveTime + speed > 6){
                    float t6 = 6 - speed;
                    float d6 = .5*(t6*t6) + speed*t6;
                    driveTime = t6 + ((distance - buffer - d6)/6);
                }
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * (1/rate) * .26));
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
                driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
                // If it will accelerate too fast
                if (driveTime + speed > 6){
                    float t6 = 6 - speed;
                    float d6 = .5*(t6*t6) + speed*t6;
                    driveTime = t6 + ((distance - buffer - d6)/6);
                }
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * (1/rate) * .26));
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
            //1. NR
            printf("speed: %f for car: carId: %i\n", speed, carId);

            for (int iX = 0; iX < 5; iX++) {
                for (int iY = 5; iY < 10; iY++) {
                    tanValue = (10-iY)/((float)(10 - iX));
                    angle = atan(tanValue);
                    if(angle < 0){
                      printf("1nr angle negative\n");
                    }
                    r = sqrt(pow((10-iY), 2) + pow((10 - iX),2));
                    driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
                    // If it will accelerate too fast
                    if (driveTime + speed > 6){
                        float t6 = 6 - speed;
                        float d6 = .5*(t6*t6) + speed*t6;
                        driveTime = t6 + ((distance + buffer - d6)/6);
                    }
                    float intersectionSpeed = speed + driveTime;
                    timeWillLeave = tm + driveTime + (angle*r+(buffer * (1/rate) * .1))/intersectionSpeed;
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
                    driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
                    // If it will accelerate too fast
                    if (driveTime + speed > 6){
                        float t6 = 6 - speed;
                        float d6 = .5*(t6*t6) + speed*t6;
                        driveTime = t6 + ((distance + buffer - d6)/6);
                    }
                    float intersectionSpeed = speed + driveTime;
                    timeWillLeave = tm + driveTime + (angle*r+(buffer * (1/rate) * .1))/intersectionSpeed;
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
                    driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
                    // If it will accelerate too fast
                    if (driveTime + speed > 6){
                        float t6 = 6 - speed;
                        float d6 = .5*(t6*t6) + speed*t6;
                        driveTime = t6 + ((distance + buffer - d6)/6);
                    }
                    float intersectionSpeed = speed + driveTime;
                    timeWillLeave = tm + driveTime + (angle*r+(buffer * (1/rate) * .1))/intersectionSpeed;
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            printf ("speedIntent: %f",speedIntent);
            printf("timeWillLeave: %f\n", timeWillLeave);
            printf("tm: %f, driveTime: %f\n", tm, driveTime);
            return instruction::instruction(carId, speedIntent, 1);
        }
    }

    /*
     SOUTH CASE
     */

    else if ( fromDir == 2){ //From S
        distance = std::abs(45 - y);
        //1. x: 5-10, y: 0-5
        //2. x: 3-7, y: 3-7
        //3. x: 0-5, y: 5-10
        //1.SC
        for (int iX = 5; iX < 10; iX++) {
            for (int iY = 0; iY < 5; iY++) {
                tanValue = iY/((float)iX);
                angle = atan(tanValue);
                r = sqrt(pow(iX, 2) + pow(iY,2));
                driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
                // If it will accelerate too fast
                if (driveTime + speed > 6){
                    float t6 = 6 - speed;
                    float d6 = .5*(t6*t6) + speed*t6;
                    driveTime = t6 + ((distance - buffer - d6)/6);
                }
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * (1/rate) * .26));
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
                driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
                // If it will accelerate too fast
                if (driveTime + speed > 6){
                    float t6 = 6 - speed;
                    float d6 = .5*(t6*t6) + speed*t6;
                    driveTime = t6 + ((distance - buffer - d6)/6);
                }
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * (1/rate) * .26));
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
                driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
                // If it will accelerate too fast
                if (driveTime + speed > 6){
                    float t6 = 6 - speed;
                    float d6 = .5*(t6*t6) + speed*t6;
                    driveTime = t6 + ((distance - buffer - d6)/6);
                }
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * (1/rate) * .26));
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
                    driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
                    // If it will accelerate too fast
                    if (driveTime + speed > 6){
                        float t6 = 6 - speed;
                        float d6 = .5*(t6*t6) + speed*t6;
                        driveTime = t6 + ((distance + buffer - d6)/6);
                    }
                    float intersectionSpeed = speed + driveTime;
                    timeWillLeave = tm + driveTime + (angle*r+(buffer * (1/rate) * .1))/intersectionSpeed;
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            //2.SR
            for (int iX = 3; iX < 7; iX++) {
                for (int iY = 3; iY < 7; iY++) {
                    tanValue = iY/((float)iX);
                    angle = atan(tanValue);
                    r = sqrt(pow(iX, 2) + pow(iY,2));
                    driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
                    // If it will accelerate too fast
                    if (driveTime + speed > 6){
                        float t6 = 6 - speed;
                        float d6 = .5*(t6*t6) + speed*t6;
                        driveTime = t6 + ((distance + buffer - d6)/6);
                    }
                    float intersectionSpeed = speed + driveTime;
                    timeWillLeave = tm + driveTime + (angle*r+(buffer * (1/rate) * .1))/intersectionSpeed;
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            //3.SR
            for (int iX = 0; iX < 5; iX++) {
                for (int iY = 5; iY < 10; iY++) {
                    tanValue = iY/((float)iX);
                    angle = atan(tanValue);
                    r = sqrt(pow(iX, 2) + pow(iY,2));
                    driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
                    // If it will accelerate too fast
                    if (driveTime + speed > 6){
                        float t6 = 6 - speed;
                        float d6 = .5*(t6*t6) + speed*t6;
                        driveTime = t6 + ((distance + buffer - d6)/6);
                    }
                    float intersectionSpeed = speed + driveTime;
                    timeWillLeave = tm + driveTime + (angle*r+(buffer * (1/rate) * .1))/intersectionSpeed;
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            printf ("speedIntent: %f",speedIntent);
            printf("timeWillLeave: %f\n", timeWillLeave);
            printf("tm: %f, driveTime: %f\n", tm, driveTime);
            return instruction::instruction(carId, speedIntent, 1);
        }
    }

    /*
     EAST CASE
     */

    else if ( fromDir == 1){ //From E
        distance = std::abs(x - 55);
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
                driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
                // If it will accelerate too fast
                if (driveTime + speed > 6){
                    float t6 = 6 - speed;
                    float d6 = .5*(t6*t6) + speed*t6;
                    driveTime = t6 + ((distance - buffer - d6)/6);
                }
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * (1/rate) * .26));
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
                driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
                // If it will accelerate too fast
                if (driveTime + speed > 6){
                    float t6 = 6 - speed;
                    float d6 = .5*(t6*t6) + speed*t6;
                    driveTime = t6 + ((distance - buffer - d6)/6);
                }
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * (1/rate) * .26));
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
                driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
                // If it will accelerate too fast
                if (driveTime + speed > 6){
                    float t6 = 6 - speed;
                    float d6 = .5*(t6*t6) + speed*t6;
                    driveTime = t6 + ((distance - buffer - d6)/6);
                }
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * (1/rate) * .26));
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
                    driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
                    // If it will accelerate too fast
                    if (driveTime + speed > 6){
                        float t6 = 6 - speed;
                        float d6 = .5*(t6*t6) + speed*t6;
                        driveTime = t6 + ((distance + buffer - d6)/6);
                    }
                    float intersectionSpeed = speed + driveTime;
                    timeWillLeave = tm + driveTime + (angle*r+(buffer * (1/rate) * .1))/intersectionSpeed;
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            //2.ER
            for (int iX = 3; iX < 7; iX++) {
                for (int iY = 3; iY < 7; iY++) {
                    tanValue = (10-iX)/((float)iY);
                    angle = atan(tanValue);
                    r = sqrt(pow((10-iX), 2) + pow(iY,2));
                    driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
                    // If it will accelerate too fast
                    if (driveTime + speed > 6){
                        float t6 = 6 - speed;
                        float d6 = .5*(t6*t6) + speed*t6;
                        driveTime = t6 + ((distance + buffer - d6)/6);
                    }
                    float intersectionSpeed = speed + driveTime;
                    timeWillLeave = tm + driveTime + (angle*r+ (buffer * (1/rate) * .1))/intersectionSpeed;
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            //3.ER
            for (int iX = 0; iX < 5; iX++) {
                for (int iY = 0; iY < 5; iY++) {
                    tanValue = (10-iX)/((float)iY);
                    angle = atan(tanValue);
                    r = sqrt(pow((10-iX), 2) + pow(iY,2));
                    driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
                    // If it will accelerate too fast
                    if (driveTime + speed > 6){
                        float t6 = 6 - speed;
                        float d6 = .5*(t6*t6) + speed*t6;
                        driveTime = t6 + ((distance + buffer - d6)/6);
                    }
                    float intersectionSpeed = speed + driveTime;
                    timeWillLeave = tm + driveTime + (angle*r+ (buffer * (1/rate) * .1))/intersectionSpeed;
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            printf ("speedIntent: %f",speedIntent);
            printf("timeWillLeave: %f\n", timeWillLeave);
            printf("tm: %f, driveTime: %f\n", tm, driveTime);
            return instruction::instruction(carId, speedIntent, 1);
        }
    }

    /*
     WEST CASE
     */

    else if ( fromDir == 3){ //From W
        distance = std::abs(45 - x);
        //1. x: 0-5, y: 0-5
        //2. x: 3-7, y: 3-7
        //3. x: 5-10, y: 5-10
        //1. WC
        for (int iX = 0; iX < 5; iX++) {
            for (int iY = 0; iY < 5; iY++) {
                tanValue = (iX)/((float)(10-iY));
                angle = atan(tanValue);
                r = sqrt(pow((iX), 2) + pow((10-iY),2));
                driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
                // If it will accelerate too fast
                if (driveTime + speed > 6){
                    float t6 = 6 - speed;
                    float d6 = .5*(t6*t6) + speed*t6;
                    driveTime = t6 + ((distance - buffer - d6)/6);
                }
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    printf("1 wc twr:%f\n", timeWillReach);
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * (1/rate) * .26));
                    if (idealSpeed < 0){
                        if(distance < 0){
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        printf("numerator: %f\n", ((float)distance + (angle*r) - buffer));
                        printf("denominator: %f\n", (sdcManager::getGrid(iX,iY) - tm));
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
                driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
                // If it will accelerate too fast
                if (driveTime + speed > 6){
                    float t6 = 6 - speed;
                    float d6 = .5*(t6*t6) + speed*t6;
                    driveTime = t6 + ((distance - buffer - d6)/6);
                }
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    printf("2 wc twr:%f\n", timeWillReach);
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * (1/rate) * .26));
                    if (idealSpeed < 0){
                        if(distance < 0){
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        printf("idealSpeed: %f\n", idealSpeed);
                        printf("speedIntent3: %f\n", speedIntent);
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
                driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
                // If it will accelerate too fast
                if (driveTime + speed > 6){
                    float t6 = 6 - speed;
                    float d6 = .5*(t6*t6) + speed*t6;
                    driveTime = t6 + ((distance - buffer - d6)/6);
                }
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                    reserved = false;
                    printf("3 wc twr:%f\n", timeWillReach);
                    return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance + (angle*r)) / (sdcManager::getGrid(iX,iY) - tm + (buffer * (1/rate) * .26));
                    if (idealSpeed < 0){
                        if(distance < 0){
                            printf("negative speedIntent distance\n");
                            return instruction::instruction(carId, decrease*speed, 0);
                        }
                    }
                    else{
                        printf("idealSpeed: %f\n", idealSpeed);
                        printf("speedIntent3: %f\n", speedIntent);
                        speedIntent = fmin(speedIntent, idealSpeed);
                    }
                }
            }
        }
        if (speedIntent < 3){
            printf("wc speedIntent:%f\n", speedIntent);
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
                    driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
                    // If it will accelerate too fast
                    if (driveTime + speed > 6){
                        float t6 = 6 - speed;
                        float d6 = .5*(t6*t6) + speed*t6;
                        driveTime = t6 + ((distance + buffer - d6)/6);
                    }
                    float intersectionSpeed = speed + driveTime;
                    timeWillLeave = tm + driveTime + (angle*r + (buffer * (1/rate) * .1))/intersectionSpeed;
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
                    driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
                    // If it will accelerate too fast
                    if (driveTime + speed > 6){
                        float t6 = 6 - speed;
                        float d6 = .5*(t6*t6) + speed*t6;
                        driveTime = t6 + ((distance + buffer - d6)/6);
                    }
                    float intersectionSpeed = speed + driveTime;
                    timeWillLeave = tm + driveTime + (angle*r +(buffer * (1/rate) * .1))/intersectionSpeed;
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
                    driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
                    // If it will accelerate too fast
                    if (driveTime + speed > 6){
                        float t6 = 6 - speed;
                        float d6 = .5*(t6*t6) + speed*t6;
                        driveTime = t6 + ((distance + buffer - d6)/6);
                    }
                    float intersectionSpeed = speed + driveTime;
                    timeWillLeave = tm + driveTime + (angle*r+(buffer * (1/rate) * .1))/intersectionSpeed;
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            printf ("speedIntent: %f",speedIntent);
            printf("timeWillLeave: %f\n", timeWillLeave);
            printf("tm: %f, driveTime: %f\n", tm, driveTime);
            return instruction::instruction(carId, speedIntent, 1);
        }
    }
    return instruction::instruction(carId, speed*.2, 0);

}

instruction sdcManager::rightTurnRequest(int carId, float x, float y, float speed, int turning, int direction, int fromDir){
    if(speed < 1){
        speed = 1;
    }
    std::chrono::milliseconds currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    int diffTime = currentTime.count() - msStartTime.count();
    float tm = diffTime *rate / float(1000);
    float decrease = .95;
    bool reserved = true;
    float driveTime = 0;
    float idealSpeed = 0;
    float buffer = fmax(4.25, 4.25 + speed);
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

    if (fromDir == 0) { //From N
        distance = std::abs(y - 55);
        //Checking if we can make reservation
        for (int iX = 0; iX < 4; iX++) { //loop x
            for (int iY = 6; iY < 10; iY++) { //loop y
                tanValue = (10-iY)/((float)iX);
                angle = atan(tanValue);
                r = sqrt(pow((10-iY), 2) + pow(iX,2));
                driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
                // If it will accelerate too fast
                if (driveTime + speed > 6){
                    float t6 = 6 - speed;
                    float d6 = .5*(t6*t6) + speed*t6;
                    driveTime = t6 + ((distance - buffer - d6)/6);
                    //driveTime = (pow(speed,2) - (10*speed) + (2*(distance + buffer) + 36))/12;
                }
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                  reserved = false;
                  //printf("car rejected: %i\n", carId);
                 // printf("gridTime: %f\n", sdcManager::getGrid(iX, iY));
                 // printf("time will reach: %f\n", timeWillReach);
                  return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance - buffer) / (sdcManager::getGrid(iX, iY) - tm);
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
                for (int iY = 6; iY < 10; iY++) { //loop y
                    tanValue = (10-iY)/((float)iX);
                    angle = atan(tanValue);
                    r = sqrt(pow((10-iY), 2) + pow(iX,2));
                    driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
                    // If it will accelerate too fast
                    if (driveTime + speed > 6){
                        float t6 = 6 - speed;
                        float d6 = .5*(t6*t6) + speed*t6;
                        driveTime = t6 + ((distance + buffer - d6)/6);
                        //driveTime = (pow(speed,2) - (10*speed) + (2*(distance + buffer) + 36))/12;
                    }
                    timeWillLeave = tm + driveTime + (angle*r)/speed;
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            printf ("speedIntent: %f",speedIntent);
            printf("timeWillLeave: %f\n", timeWillLeave);
            printf("tm: %f, driveTime: %f\n", tm, driveTime);
            return instruction::instruction(carId, speedIntent, 1);
        }
    }

    /*
    SOUTH CASE
    */

    else if ( fromDir == 2){ //From S
        distance = std::abs(45 - y);
        //Checking if we can make reservation
        for (int iX = 6; iX < 10; iX++) { //loop x
            for (int iY = 0; iY < 4; iY++) { //loop y
                tanValue = ((float)iY)/(10 - iX);
                angle = atan(tanValue);
                r = sqrt(pow(iY, 2) + pow((10 - iX),2));
                driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
                // If it will accelerate too fast
                if (driveTime + speed > 6){
                    float t6 = 6 - speed;
                    float d6 = .5*(t6*t6) + speed*t6;
                    driveTime = t6 + ((distance - buffer - d6)/6);
                    //driveTime = (pow(speed,2) - (10*speed) + (2*(distance + buffer) + 36))/12;
                }
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                  reserved = false;
                 // printf("car rejected: %i\n", carId);
                  //printf("gridTime: %f\n", sdcManager::getGrid(iX, iY));
                  //printf("time will reach: %f\n", timeWillReach);
                  return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance - buffer) / (sdcManager::getGrid(iX, iY) - tm);
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
                for (int iY = 0; iY < 4; iY++) { //loop y
                    tanValue = (10-iY)/((float)iX);
                    angle = atan(tanValue);
                    r = sqrt(pow(iY, 2) + pow((10 - iX),2));
                    driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
                    // If it will accelerate too fast
                    if (driveTime + speed > 6){
                        float t6 = 6 - speed;
                        float d6 = .5*(t6*t6) + speed*t6;
                        driveTime = t6 + ((distance + buffer - d6)/6);
                        //driveTime = (pow(speed,2) - (10*speed) + (2*(distance + buffer) + 36))/12;
                    }
                    timeWillLeave = tm + driveTime + (angle*r)/speed;
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            printf("carId: %i got reservation\n",carId);
            printf ("speedIntent: %f",speedIntent);
            printf("timeWillLeave: %f\n", timeWillLeave);
            printf("tm: %f, driveTime: %f\n", tm, driveTime);
            return instruction::instruction(carId, speedIntent, 1);
        }
    }

    /*
    EAST CASE
    */

    else if ( fromDir == 1){ //From E
        distance = std::abs(x - 55);
        //Checking if we can make reservation
        for (int iX = 6; iX < 10; iX++) { //loop x
            for (int iY = 6; iY < 10; iY++) { //loop y
                tanValue = (10-iX)/((float)(10 - iY));
                angle = atan(tanValue);
                r = sqrt(pow((10 - iY), 2) + pow((10 - iX),2));
                driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
                // If it will accelerate too fast
                if (driveTime + speed > 6){
                    float t6 = 6 - speed;
                    float d6 = .5*(t6*t6) + speed*t6;
                    driveTime = t6 + ((distance - buffer - d6)/6);
                    //driveTime = (pow(speed,2) - (10*speed) + (2*(distance + buffer) + 36))/12;
                }
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                  reserved = false;
                  //printf("car rejected: %i\n", carId);
                 // printf("gridTime: %f\n", sdcManager::getGrid(iX, iY));
                 // printf("time will reach: %f\n", timeWillReach);
                  return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance - buffer) / (sdcManager::getGrid(iX, iY) - tm);
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
                for (int iY = 6; iY < 10; iY++) { //loop y
                    tanValue = (10-iX)/((float)(10 - iY));
                    angle = atan(tanValue);
                    r = sqrt(pow((10 - iY), 2) + pow((10 - iX),2));
                    driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
                    // If it will accelerate too fast
                    if (driveTime + speed > 6){
                        float t6 = 6 - speed;
                        float d6 = .5*(t6*t6) + speed*t6;
                        driveTime = t6 + ((distance + buffer - d6)/6);
                        //driveTime = (pow(speed,2) - (10*speed) + (2*(distance + buffer) + 36))/12;
                    }
                    timeWillLeave = tm + driveTime + (angle*r)/speed;
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            printf("carId: %i got reservation\n",carId);
            printf ("speedIntent: %f",speedIntent);
            printf("timeWillLeave: %f\n", timeWillLeave);
            printf("tm: %f, driveTime: %f\n", tm, driveTime);
            return instruction::instruction(carId, speedIntent, 1);
        }
    }

    /*
    WEST CASE
    */

    else if ( fromDir == 3){ //From W
        distance = std::abs(45 - x);
        //Checking if we can make reservation
        for (int iX = 0; iX < 4; iX++) { //loop x
            for (int iY = 0; iY < 4; iY++) { //loop y
                tanValue = iX/((float)iY);
                angle = atan(tanValue);
                r = sqrt(pow(iY, 2) + pow(iX,2));
                driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance - 2*buffer));
                // If it will accelerate too fast
                if (driveTime + speed > 6){
                    float t6 = 6 - speed;
                    float d6 = .5*(t6*t6) + speed*t6;
                    driveTime = t6 + ((distance - buffer - d6)/6);
                    //driveTime = (pow(speed,2) - (10*speed) + (2*(distance + buffer) + 36))/12;
                }
                timeWillReach = tm + driveTime + (angle*r)/speed;
                if (timeWillReach < sdcManager::getGrid(iX, iY) || !reserved) { //compare it
                  reserved = false;
                 // printf("car rejected: %i\n", carId);
                 // printf("gridTime: %f\n", sdcManager::getGrid(iX, iY));
                 // printf("time will reach: %f\n", timeWillReach);
                  return instruction::instruction(carId, speed*decrease, 0);
                }
                else {
                    idealSpeed = ((float)distance - buffer) / (sdcManager::getGrid(iX, iY) - tm);
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
                for (int iY = 0; iY < 4; iY++) { //loop y
                    tanValue = iX/((float)iY);
                    angle = atan(tanValue);
                    r = sqrt(pow(iY, 2) + pow(iX,2));
                    //printf(" arc length: %f \n", (angle*r));
                    float leaveSpeed = fmin(speed, maxTurnRight);
                    driveTime = sdcManager::quadraticRoot(1, 2*speed, -2*(distance + 2*buffer));
                    // If it will accelerate too fast
                    if (driveTime + speed > 6){
                        float t6 = 6 - speed;
                        float d6 = .5*(t6*t6) + speed*t6;
                        driveTime = t6 + ((distance + buffer - d6)/6);
                        //driveTime = (pow(speed,2) - (10*speed) + (2*(distance + buffer) + 36))/12;
                    }
                    timeWillLeave = tm + driveTime + (angle*r)/leaveSpeed;
                    printf ("speedIntent: %f",speedIntent);
                    printf("timeWillLeave: %f\n", timeWillLeave);
                    printf("tm: %f, driveTime: %f\n", tm, driveTime);
                    sdcManager::setGrid(timeWillLeave, iX, iY);
                }
            }
            printf("carId: %i got reservation\n",carId);
            return instruction::instruction(carId, speedIntent, 1);
        }
    }
    return instruction::instruction(carId, speed*.2, 0);
}

bool sdcManager::stopSignHandleRequest(int carId, int turning, int direction, int fromDir) {
    //    printf("carAmt: %i\n", carAmt);
    //    printf("turning: %i\n", turning);
    fflush(stdout);

    if (carAmt == 0) { //no car at intersection
        registerCar(carId, turning, direction);
        carNorthQueue.erase(std::remove(carNorthQueue.begin(), carNorthQueue.end(), carId), carNorthQueue.end());
        carEastQueue.erase(std::remove(carEastQueue.begin(), carEastQueue.end(), carId), carEastQueue.end());
        carSouthQueue.erase(std::remove(carSouthQueue.begin(), carSouthQueue.end(), carId), carSouthQueue.end());
        carWestQueue.erase(std::remove(carWestQueue.begin(), carWestQueue.end(), carId), carWestQueue.end());
        return true;
    }else if(carAmt == 1) { //one car already in intersection
        switch(currentTurn){
                //car in intersection is going straight
            case 0 :
                if (turning == 0 && (direction + 2)%4 == currentDir) {
                    registerCar(carId, turning, direction);

                    //removing from all queues even though it will only happen in 1 for shorter code
                    carNorthQueue.erase(std::remove(carNorthQueue.begin(), carNorthQueue.end(), carId), carNorthQueue.end());
                    carEastQueue.erase(std::remove(carEastQueue.begin(), carEastQueue.end(), carId), carEastQueue.end());
                    carSouthQueue.erase(std::remove(carSouthQueue.begin(), carSouthQueue.end(), carId), carSouthQueue.end());
                    carWestQueue.erase(std::remove(carWestQueue.begin(), carWestQueue.end(), carId), carWestQueue.end());
                    return true;
                }
                else if (turning == 2 && ((direction + 2)%4 == currentDir || (direction + 3)%4 == currentDir)) {
                    printf("other car turning right\n");
                    registerCar(carId, turning, direction);
                    carNorthQueue.erase(std::remove(carNorthQueue.begin(), carNorthQueue.end(), carId), carNorthQueue.end());
                    carEastQueue.erase(std::remove(carEastQueue.begin(), carEastQueue.end(), carId), carEastQueue.end());
                    carSouthQueue.erase(std::remove(carSouthQueue.begin(), carSouthQueue.end(), carId), carSouthQueue.end());
                    carWestQueue.erase(std::remove(carWestQueue.begin(), carWestQueue.end(), carId), carWestQueue.end());
                    return true;
                }
                else{
                    return false;
                }


                //car in intersection is going left
            case 1:
                return false;
                break;
                //car in intersection is going right
            case 2:
                if (turning == 1) {
                    return false;
                }else if (turning == 2) { //can turn if prev car turning right
                    registerCar(carId, turning, direction);
                    carNorthQueue.erase(std::remove(carNorthQueue.begin(), carNorthQueue.end(), carId), carNorthQueue.end());
                    carEastQueue.erase(std::remove(carEastQueue.begin(), carEastQueue.end(), carId), carEastQueue.end());
                    carSouthQueue.erase(std::remove(carSouthQueue.begin(), carSouthQueue.end(), carId), carSouthQueue.end());
                    carWestQueue.erase(std::remove(carWestQueue.begin(), carWestQueue.end(), carId), carWestQueue.end());
                    return true;
                }else{
                    if (!(direction == currentDir)) {
                        registerCar(carId, turning, direction);
                        carNorthQueue.erase(std::remove(carNorthQueue.begin(), carNorthQueue.end(), carId), carNorthQueue.end());
                        carEastQueue.erase(std::remove(carEastQueue.begin(), carEastQueue.end(), carId), carEastQueue.end());
                        carSouthQueue.erase(std::remove(carSouthQueue.begin(), carSouthQueue.end(), carId), carSouthQueue.end());
                        carWestQueue.erase(std::remove(carWestQueue.begin(), carWestQueue.end(), carId), carWestQueue.end());
                        return true;
                    }
                    return false;
                }
            case 3:
                return false;

        }
    }

    else { //more than 1 car
        return false;
    }
    return false;
}

float sdcManager::quadraticRoot(float a, float b, float c){
    float D = sqrt((b*b) - (4*a*c));
    //printf("a:%f b:%f c:%f\n",a,b,c);
    //printf("d: %f\n", D);
    if (!a)// a==0 => 2*a==0
    {
        printf("divide by zero\n");
        return 0;
    }
    return ( -b + D)/(2*a);;//function succeded only return larger root
}
/*
 turn right case:
 destdirection = 0
 if(!nextcar turn left && ! nextcar straight north):
 give reservation

 straight case:
 destdirection = 0
 if(nextcar straight 2 || nextcar right east || nextcar right south
 */
bool sdcManager::stopSignQueue(int carId, int fromDir) {
    printf("in stopsignqueue\n");
    switch (fromDir) {
        case 0:
            // printf("north queue \n\n\n");
            carNorthQueue.push_back(carId);
            break;
        case 1:
            // printf("east queue \n\n\n");
            carEastQueue.push_back(carId);
            break;
        case 2:
            //  printf("south queue \n\n\n");
            carSouthQueue.push_back(carId);
            break;
        case 3:
            //  printf("west queue \n\n\n");
            carWestQueue.push_back(carId);
            break;
    }
    return false;

}
bool sdcManager::shouldStop(int carId, int fromDir) {
    switch (fromDir) {
        case 0:
            if(carNorthQueue.size() > 1){
                //printf("north queue\n\n\n");
                if(nStop){
                    return true;
                }
                else{
                    return false;
                }
            }
            else{
                return false;
            }

            break;
        case 1:
            if(carEastQueue.size() > 1){
                //printf("east queue\n\n\n");
                if(eStop){
                    //printf("eStop true\n\n");
                    return true;
                }
                else{
                    return false;
                }
            }
            else{
                return false;
            }

            break;
        case 2:
            if(carSouthQueue.size() > 1){
                //printf("south queue\n\n\n");
                if(sStop){
                    return true;
                }
                else{
                    return false;
                }
            }
            else{
                return false;
            }

            break;
        case 3:
            if(carWestQueue.size() > 1){
                // printf("west queue\n\n\n");
                if(wStop){
                    return true;
                }
                else{
                    return false;
                }
            }
            else{
                return false;
            }

            break;

    }
    return false;

}
void sdcManager::stopSignCarLeft(int carId) {
    carList.erase(std::remove(carList.begin(), carList.end(), carId), carList.end());
    carAmt--;
}

gazebo::sdcSensorData *sdcManager::getSensorData(int cameraId) {
    //HOW TO FIX: USE A MAP OF sdcSensorDatas WHERE THE KEY IS THE cameraId
    // If its not in the map then make a new sensor data. Otherwise return it.

    if (sensorManager.count(cameraId) > 0){
        return &sensorManager[cameraId];
    }
    else{
        sensorManager.insert(std::pair<int, gazebo::sdcSensorData>(cameraId, gazebo::sdcSensorData(cameraId)));
        printf("map count: %lu\n",sensorManager.count(cameraId));
        printf("cameraId in manager: %i\n\n",cameraId);
        return &sensorManager[cameraId];
    }


    /*

     // printf("in manager\n");
     //printf("car id: %i\n", carId);
     fflush(stdout);

     //printf("sensorList size is: %lu while car id is: %d\n",sensorDataList.size(), carId);
     if (sensorDataList.size() < cameraId) {
     //  printf("in manager and creating sensordata\n");
     gazebo::sdcSensorData temp = gazebo::sdcSensorData(cameraId);
     //temp.UpdateSteeringMagnitude(0);
     sensorDataList.push_back(temp);

     }
     //printf("car Id: %i", carId);
     //printf("sensorId is: %d\n",sensorDataList.at(carId-1)->sensorId);
     //   gazebo::sdcSensorData tempSensor = sensorDataList.at(carId-1);
     //    sensorDataList.at(carId-1).UpdateSteeringMagnitude(0.0);
     return &sensorDataList.at(cameraId-1);
     */
}
