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
    printf("created manager\n");
    printf("%d\n", id);
    sdcManager::makeGrid();
    //sdcManager::setGrid(1, 2, 3);
    //printf("Spot 2,3: %i\n", sdcManager::getGrid(2,3));
    //printf("Spot 3,3: %i\n", sdcManager::getGrid(3,3));
}

void sdcManager:: printid(){
    printf("id is: ");
    printf("%d", id);
}

void sdcManager::makeGrid(){
    time_t tm = difftime(time(0), startTime);
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
    printf("registered, and now carAmt is: %i\n", carAmt);
    fflush(stdout);
    carList.push_back(carId);
    printf("registered car Id is: %i\n", carId);
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
    //10 is the normal car max speed
  //  printf("start of request carId: %d\n", fromDir);
    time_t tm = difftime(time(0),startTime);
    //printf("time: %ld\n", tm);
    if (speed == 0){
      speed = 1;
    }
    float speedIntent = 10;

    //float tm = 1;
    bool reserved = true;
    if(turning == 0){
      float timeWillTake = 0;
      float timeWillLeave = 0;
      float distance = 0;
      if (fromDir == 0 || fromDir == 2) { //travelling y bound
        int xIndex = static_cast<int>(x-46);
        //printf("carId is y bound: %d\n", carId);
        for (int yIndex = 0; yIndex < gridSize; yIndex++) {
          distance = std::abs(yIndex + 45 - y);
          timeWillTake = tm + (fmax(0,((float)distance - speed*2)) / speed);
          //printf("distance: %f\n",fmax(0,((float)distance - speed*2)));
          //timeWillLeave = tm + ((float)distance - speed) / speed;
        //  printf("car speed: %f\n\n", speed);
          //printf("car old and reach diff: %f, %f\n\n", sdcManager::getGrid(xIndex, yIndex), timeWillTake);
          for (int xWidth = 0; xWidth < 3; xWidth ++) { //xWidth is width buffer
            if (timeWillTake < sdcManager::getGrid(xIndex + xWidth, yIndex) || !reserved) {
              reserved = false;
            //  printf("car rrejected: %d\n", carId);
              return instruction::instruction(carId, .2*speed, 0);
            } else {
              speedIntent = fmin(speedIntent, ((float)distance - speed) / (sdcManager::getGrid(xIndex + xWidth, yIndex) - tm));
              if (speedIntent < 0){
                speedIntent = 10;
              }
            }
          }
        }

        if (speedIntent < 2){
          return instruction::instruction(carId, .2*speed, 0);
        }
        if (reserved) {
          for (int yIndex = 0; yIndex < gridSize; yIndex++) {
            distance = std::abs(yIndex + 45 - y);
            //timeWillTake = tm + (distance - (speed * 1.5)) / speed;
            timeWillLeave = tm + (distance + (speedIntent * 1.5)) / speedIntent;
            for (int xWidth = 0; xWidth < 3; xWidth ++) { //xWidth is width buffer
                sdcManager::setGrid(timeWillLeave, xIndex + xWidth, yIndex);
            }
          }
        //  printf("carId got reserved: %d\n", carId);
            //printf("carId intend: %d, %f\n", carId, speedIntent);
            return instruction::instruction(carId, speedIntent, 1);
          }

      }
      else if (fromDir == 1 || fromDir == 3) { //travelling x bound
      //  printf("carId is x bound: %d\n", carId);
        int yIndex = static_cast<int>(y-46);
        for (int xIndex = 0; xIndex < gridSize; xIndex++) {
          distance = std::abs(xIndex + 45 - x);
          timeWillTake = tm + (fmax(0,((float)distance - speed*2)) / speed);
          //timeWillLeave = tm + ((float)distance - speed) / speed;

          //printf("car old leave diff: %f, %f\n", sdcManager::getGrid(xIndex, yIndex), timeWillTake);
          for (int yWidth = 0; yWidth < 3; yWidth ++) { //xWidth is width buffer
            if (timeWillTake < sdcManager::getGrid(xIndex, yIndex + yWidth) || !reserved) {
              reserved = false;
            //  printf("car rejected: %d\n", carId);
              return instruction::instruction(carId, speed*.2, 0);
            } else {

              speedIntent = fmin(speedIntent, ((float)distance - speed) / (sdcManager::getGrid(xIndex, yIndex + yWidth) - tm));
              //printf("gridTime: %f\n", sdcManager::getGrid(xIndex, yIndex + yWidth));
            //  printf("tm: %ld\n", tm);
            //  printf("carId intend: %d, %f\n", carId, speedIntent);
              if (speedIntent < 0){
                speedIntent = 10;
              }

              //speed = fmax(speed, speedIntent);
            }
          }

        }
        if(speedIntent < 2){
          return instruction::instruction(carId, speed*.2, 0);
        }

        if (reserved) {
          for (int xIndex = 0; xIndex < gridSize; xIndex++) {
            distance = std::abs(xIndex + 45 - x);
            //timeWillTake = tm + (distance - speed) / speed;
            timeWillLeave = tm + (distance + (speedIntent*1.5)) / speedIntent;
            for (int yWidth = 0; yWidth < 3; yWidth ++) {
                sdcManager::setGrid(timeWillLeave, xIndex, yIndex + yWidth);
            }
          }
          // if(speed < 1){
          //   speed = 10;
          // }
          //printf("carId got reserved: %d, %f\n", carId, speedIntent);
          return instruction::instruction(carId, speedIntent, 1);
        }
      }
      return instruction::instruction(carId, 0, 0);
    }
    //left?
    else if (turning == 1){
    //  printf("maxTurnLeft: %f\n", sdcManager::maxTurnLeft);
        return instruction::instruction(carId, sdcManager::maxTurnLeft, 1);
    }
    else if (turning == 2){
        return instruction::instruction(carId, sdcManager::maxTurnRight, 1);
    }
    else{
      return instruction::instruction(carId, speed*.9, 0);
    }


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
