#ifndef SDCMANAGER_HH_
#define SDCMANAGER_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>

#include <iostream>
#include <vector>
#include <time.h>
#include "request.hh"
#include "instruction.hh"
#include "sdcSensorData.hh"

class sdcManager{
public:
    sdcManager(int id);
    int id;
    void printid();
    static void registerCar(int carId, int turning, int direction);
    static void laneStopRequest(int fromDir);

    //reservation
    void makeGrid();
    static void setGrid(float filled, int x, int y);
    static float getGrid(int x, int y);

    static instruction reservationRequest(int carId, float x, float y, float speed, int turning, int direction, int fromDir);
    static bool stopSignHandleRequest(int carId, int turning, int direction, int fromDir);
    static void stopSignCarLeft(int carId);
    static bool stopSignQueue(int carId, int fromDir);
    static bool shouldStop(int carId, int fromDir);
    static gazebo::sdcSensorData *getSensorData(int cameraId);
private:
    static int carAmt;
    static std::vector<int> carList;
    static std::vector<std::vector<float>> grid;
    static float maxTurnLeft;
    static float maxTurnRight;






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
    static int gridSize;


};
#endif
