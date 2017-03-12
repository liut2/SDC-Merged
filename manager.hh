

#ifndef MANAGER_HH_
#define MANAGER_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>

#include <iostream>
#include <vector>
#include "instruction.hh"
#include "sdcSensorData.hh"
/***
*
* The manager class implements the stop sign protocol and policy
* It also is used to connect cars with their sensors through sensorData objects
***/
class manager{
    public:
        //id incase you add more than 1 intersection
        manager(int id);
        int id;
        static void registerCar(int carId, int turning, int direction);
        //Used in stop sign policy to prevent cars from hitting each other before the intersection
        static void laneStopRequest(int fromDir);
        //Gives reservations to cars in the stop sign protocol
        static bool stopSignHandleRequest(int carId, int turning, int direction, int fromDir);
        //Called when a car left the intersection
        static void stopSignCarLeft(int carId);
        static bool stopSignQueue(int carId, int fromDir);
        static bool shouldStop(int carId, int fromDir);
        //Connects cars and sensors to their shared sensorData objects
        static gazebo::sdcSensorData *getSensorData(int cameraId);
    private:
        static int carAmt;
        static std::vector<int> carList;
        static std::vector<int> carNorthQueue;
        static std::vector<int> carEastQueue;
        static std::vector<int> carSouthQueue;
        static std::vector<int> carWestQueue;
        static std::vector<gazebo::sdcSensorData> sensorDataList;
        static std::map<int, gazebo::sdcSensorData> sensorManager;
        static int currentDir; //0 north, 1 east, 2 south, 3 west
        static int currentTurn; // 0 straight, 1 left, 2 right
        static bool nStop;
        static bool eStop;
        static bool sStop;
        static bool wStop;



};
#endif
