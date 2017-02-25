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
}

void manager:: printid(){
}

void manager::registerCar(int carId, int turning, int direction) {
    carAmt++;
    currentDir = direction;
    currentTurn = turning;
    fflush(stdout);
    carList.push_back(carId);
    fflush(stdout);

}
void manager::laneStopRequest(int fromDir){
    switch (fromDir) {
        case 0:
            nStop = true;
            break;
        case 1:
            eStop = true;
            break;
        case 2:
            sStop = true;
            break;
        case 3:
            wStop = true;
            break;
    }
}

bool manager::stopSignHandleRequest(int carId, int turning, int direction, int fromDir) {
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
    switch (fromDir) {
        case 0:
            carNorthQueue.push_back(carId);
            break;
        case 1:
            carEastQueue.push_back(carId);
            break;
        case 2:
            carSouthQueue.push_back(carId);
            break;
        case 3:
            carWestQueue.push_back(carId);
            break;
    }
    return false;

}
bool manager::shouldStop(int carId, int fromDir) {
    switch (fromDir) {
        case 0:
            if(carNorthQueue.size() > 1){
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
                if(eStop){
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
        return &sensorManager[cameraId];
    }


}
