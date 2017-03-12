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
#include <chrono>
#include "instruction.hh"
#include "sdcSensorData.hh"

class sdcManager{
public:
   // id in case you want to have more than 1 intersection in a world
    sdcManager(int id);
    int id;
    static void printGrid();

    //set the simulation rate and simulation time
    static void setRate(float rate);
    static void setTime(float curSimTime);
    //reservation
    void makeGrid(); //initialize a 10x10 2d vector
    static void setGrid(float newTime, int x, int y);
    static float getGrid(int x, int y);
    static int gridSize;

    //handles reservation requests and tells the car what to do
    static instruction reservationRequest(int carId, float x, float y, float speed, int turning, int direction, int fromDir);

    static float quadraticRoot(float a, float b, float c); //performs the quadratic equation
private:
    static instruction rightTurnRequest(int carId, float x, float y, float speed, int turning, int direction, int fromDir);
    static instruction leftTurnRequest(int carId, float x, float y, float speed, int turning, int direction, int fromDir);

    static std::vector<std::vector<float>> grid;
    static float maxTurnLeft; //maximum turning speed
    static float maxTurnRight; //maximum turning speed
    static float rate; //simulation rate
    static float simTime; //simulation time
    //variables for prioritized cars
    static std::map<int, float> priorityList;
    static int prioritized;
};
#endif
