#include "manager.hh"

int manager::carAmt = 0;
int manager::currentDir = 0;
int manager::currentTurn = 0;
bool manager::nStop = 0;
bool manager::eStop = 0;
bool manager::sStop = 0;
bool manager::wStop = 0;
std::vector<int> manager::carList = std::vector<int>();
std::vector<int> manager::carNorthQueue = std::vector<int>();
std::vector<int> manager::carEastQueue = std::vector<int>();
std::vector<int> manager::carSouthQueue = std::vector<int>();
std::vector<int> manager::carWestQueue = std::vector<int>();
std::vector<gazebo::sdcSensorData> manager::sensorDataList = std::vector<gazebo::sdcSensorData>();
std::map<int, gazebo::sdcSensorData> manager::sensorManager = std::map<int, gazebo::sdcSensorData>();
manager::manager(int id){
    printf("created manager");
    printf("%d", id);
}

void manager:: printid(){
    printf("id is: ");
    printf("%d", id);
}

void manager::registerCar(int carId, int turning, int direction) {
    carAmt++;
    currentDir = direction;
    currentTurn = turning;
    printf("registered, and now carAmt is: %i\n", carAmt);
    fflush(stdout);
    carList.push_back(carId);
    printf("registered car Id is: %i\n", carId);
    fflush(stdout);
    
}
void manager::laneStopRequest(int fromDir){
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

bool manager::stopSignHandleRequest(int carId, int turning, int direction, int fromDir) {
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
bool manager::stopSignQueue(int carId, int fromDir) {
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
bool manager::shouldStop(int carId, int fromDir) {
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
void manager::stopSignCarLeft(int carId) {
    carList.erase(std::remove(carList.begin(), carList.end(), carId), carList.end());
    carAmt--;
}

gazebo::sdcSensorData *manager::getSensorData(int cameraId) {
    //HOW TO FIX: USE A MAP OF sdcSensorDatas WHERE THE KEY IS THE cameraId
    // If its not in the map then make a new sensor data. Otherwise return it.
    
    if (sensorManager.count(cameraId) > 0){
        return &sensorManager[cameraId];
    }
    else{
        sensorManager.insert(std::pair<int, gazebo::sdcSensorData>(cameraId, gazebo::sdcSensorData(cameraId)));
        printf("map count: %i\n",sensorManager.count(cameraId));
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