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

//Adds a car to the carList and updates carAmt
void manager::registerCar(int carId, int turning, int direction) {
    carAmt++;
    currentDir = direction;
    currentTurn = turning;
    carList.push_back(carId);
}
//When a car reaches the intersection it tells all other cars in its lane to stop
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

//Checks if the car can get in the intersection based on its direction and turn type.
//If there is already a car in the intersection it can only go on non colliding paths with that car
bool manager::stopSignHandleRequest(int carId, int turning, int direction, int fromDir) {
    if (carAmt == 0) { //no car at intersection
        registerCar(carId, turning, direction);
        carNorthQueue.erase(std::remove(carNorthQueue.begin(), carNorthQueue.end(), carId), carNorthQueue.end());
        carEastQueue.erase(std::remove(carEastQueue.begin(), carEastQueue.end(), carId), carEastQueue.end());
        carSouthQueue.erase(std::remove(carSouthQueue.begin(), carSouthQueue.end(), carId), carSouthQueue.end());
        carWestQueue.erase(std::remove(carWestQueue.begin(), carWestQueue.end(), carId), carWestQueue.end());
        switch (fromDir) {
            case 0:
                nStop = false;
                break;
            case 1:
                eStop = false;
                break;
            case 2:
                sStop = false;
                break;
            case 3:
                wStop = false;
                break;
        }
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
                    //Tells all queues they can go again. This is fine since cars constantly make requests if they haven't gotten into
                    // the intersection yet.
                    switch (fromDir) {
                        case 0:
                            nStop = false;
                            break;
                        case 1:
                            eStop = false;
                            break;
                        case 2:
                            sStop = false;
                            break;
                        case 3:
                            wStop = false;
                            break;
                    }
                    return true;
                }
                else if (turning == 2 && ((direction + 2)%4 == currentDir || (direction + 3)%4 == currentDir)) {
                    registerCar(carId, turning, direction);
                    carNorthQueue.erase(std::remove(carNorthQueue.begin(), carNorthQueue.end(), carId), carNorthQueue.end());
                    carEastQueue.erase(std::remove(carEastQueue.begin(), carEastQueue.end(), carId), carEastQueue.end());
                    carSouthQueue.erase(std::remove(carSouthQueue.begin(), carSouthQueue.end(), carId), carSouthQueue.end());
                    carWestQueue.erase(std::remove(carWestQueue.begin(), carWestQueue.end(), carId), carWestQueue.end());
                    switch (fromDir) {
                        case 0:
                            nStop = false;
                            break;
                        case 1:
                            eStop = false;
                            break;
                        case 2:
                            sStop = false;
                            break;
                        case 3:
                            wStop = false;
                            break;
                    }
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
                    switch (fromDir) {
                        case 0:
                            nStop = false;
                            break;
                        case 1:
                            eStop = false;
                            break;
                        case 2:
                            sStop = false;
                            break;
                        case 3:
                            wStop = false;
                            break;
                    }
                    return true;
                }else{
                    if (!(direction == currentDir)) {
                        registerCar(carId, turning, direction);
                        carNorthQueue.erase(std::remove(carNorthQueue.begin(), carNorthQueue.end(), carId), carNorthQueue.end());
                        carEastQueue.erase(std::remove(carEastQueue.begin(), carEastQueue.end(), carId), carEastQueue.end());
                        carSouthQueue.erase(std::remove(carSouthQueue.begin(), carSouthQueue.end(), carId), carSouthQueue.end());
                        carWestQueue.erase(std::remove(carWestQueue.begin(), carWestQueue.end(), carId), carWestQueue.end());
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
                        return true;
                    }
                    return false;
                }
            case 3:
                return false;

        }
    }
    //We allow at most 2 cars in the interscection at once
    else { //more than 1 car
        return false;
    }
    return false;
}

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
//Stops cars in the same lane
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
//Called when a car leaves the intersection. Removes it from the carList
void manager::stopSignCarLeft(int carId) {
    carList.erase(std::remove(carList.begin(), carList.end(), carId), carList.end());
    carAmt--;
}

//This method links the sdcCar objects with their corresponding sensors.
//Since the cars cannot directly access their sensors we go through the manager.
//Each sensor sends its id, and each car sends its sensorlink id for that sensor
// (which is slightly different but we adjust depending on the sensor type)
//A sensorData object is created once for the pair and is returned.

gazebo::sdcSensorData *manager::getSensorData(int cameraId) {

    // If its not in the map then make a new sensor data. Otherwise return it.
    if (sensorManager.count(cameraId) > 0){
        return &sensorManager[cameraId];
    }
    else{
        sensorManager.insert(std::pair<int, gazebo::sdcSensorData>(cameraId, gazebo::sdcSensorData(cameraId)));
        return &sensorManager[cameraId];
    }
}
