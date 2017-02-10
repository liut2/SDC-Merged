/*
 * This class provides a central location for all data that sdcCar uses during simulation. It
 * collects data from our sensors and stores it, and provides getter and setter methods to
 * the car and sensors respectively. This class also handles converting sensor data into
 * more intuitive information, such as taking blocked sensor rays and determining what are
 * objects.
 */

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>

#include "sdcSensorData.hh"


using namespace gazebo;

/*
 * Initializes the lidar in the given position to store its minimum angle, as well as the resolution
 */
sdcSensorData::sdcSensorData() {

}
sdcSensorData::sdcSensorData(int id) {
    this->sensorId = id;
    // Angle information for each lidar
    this->lidarInfoMap = std::map<LidarPos, sdcLidarSensorInfo>();

    this->backMinAngle = sdcAngle(0);
    this->backAngleResolution = 0;

    // The rays from each lidar
    this->frontLidarRays = new std::vector<double>();
    this->backLidarRays = new std::vector<double>();

    this->topLeftLidarRays = new std::vector<double>();
    this->topRightLidarRays = new std::vector<double>();
    this->topForwardLidarRays = new std::vector<double>();
    this->topBackwardLidarRays = new std::vector<double>();

    this->sideLeftFrontLidarRays = new std::vector<double>();
    this->sideLeftBackLidarRays = new std::vector<double>();
    this->sideRightFrontLidarRays = new std::vector<double>();
    this->sideRightBackLidarRays = new std::vector<double>();

    // Camera variables
    this->stopSignFrameCount = 0;
    this->sizeOfStopSign = 0;
    this->stopSignInLeftCamera = false;
    this->stopSignInRightCamera = false;
    this->lanePosition = 0;
    this->newSteerMagnitude = 0;

    // GPS variables
    this->gpsX = 0;
    this->gpsY = 0;
    this->gpsYaw = sdcAngle(0);
}

void sdcSensorData::InitLidar(LidarPos lidar, double minAngle, double angleResolution, double maxRange, int numRays){
    printf("init lidar\n");

    switch (lidar) {
        case TOP:
        case SIDE_LEFT:
        case SIDE_RIGHT:
        // Don't init for enums that correspond to more than one sensor
        break;
        default:
        this->lidarInfoMap[lidar] = sdcLidarSensorInfo(sdcAngle(minAngle), angleResolution, maxRange, numRays);

    }
}

/*
 * Updates the lidar in the given position to hold the given rays
 */
void sdcSensorData::UpdateLidar(LidarPos lidar, std::vector<double>* newRays){
    lidarInfoMap[lidar].lastUpdate = (lidarInfoMap[lidar].lastUpdate + 1) % 100000000;
    //printf("created new lidar rays\n");
    switch (lidar) {
        case FRONT:

        this->frontLidarRays = newRays;
        break;

        case BACK:
        this->backLidarRays = newRays;
        break;

        case TOP_FORWARD:
        this->topForwardLidarRays = newRays;
        break;

        case TOP_RIGHT:
        this->topRightLidarRays = newRays;
        break;

        case TOP_BACKWARD:
        this->topBackwardLidarRays = newRays;
        break;

        case TOP_LEFT:
        this->topLeftLidarRays = newRays;
        break;

        case SIDE_LEFT_FRONT:
        this->sideLeftFrontLidarRays = newRays;
        break;

        case SIDE_LEFT_BACK:
        this->sideLeftBackLidarRays = newRays;
        break;

        case SIDE_RIGHT_FRONT:
        this->sideRightFrontLidarRays = newRays;
        break;

        case SIDE_RIGHT_BACK:
        this->sideRightBackLidarRays = newRays;
        break;

        default:
        // The given enum matches more than one lidar sensor, to be safe we'll ignore it
        break;
    }
}

/*
 * Retrieve any camera data that we might find helpful
 */
void sdcSensorData::UpdateCameraData(int lanePos) {

    this->lanePosition = lanePos;
}

int sdcSensorData::LanePosition() {
    return this->lanePosition;
}

double sdcSensorData::getMidlineAngle() {
  return this->midlineAngle;
}

void sdcSensorData::setMidlineAngle(double midlineAngle) {
  this->midlineAngle = midlineAngle;
}

double sdcSensorData::getVerticalDifference() {
  return this->verticalDifference;
}

void sdcSensorData::setVerticalDifference(double verticalDifference) {
  this->verticalDifference = verticalDifference;
}

void sdcSensorData::UpdateSteeringMagnitude(double steerMag) {
  //  printf("newSteerMagnitude: %f\n", this->newSteerMagnitude);
   // printf("steerMag: %f\n", steerMag);
   // printf("Update mag sensorID: %i\n", this->sensorId);
    this->newSteerMagnitude = steerMag;
    //printf("newSteerMagnitude: %f\n", this->newSteerMagnitude);
}

double sdcSensorData::GetNewSteeringMagnitude() {
    //printf("sensorData steering mag: %f\n", this->newSteerMagnitude);
    //printf("GET mag sensorID: %i\n", this->sensorId);
    return this->newSteerMagnitude;
    //printf("newSteerMagnitude: %f\n", this->newSteerMagnitude);
}

/*
 * Get the last update tick for the given lidar
 */
int sdcSensorData::GetLidarLastUpdate(LidarPos lidar){
    return this->lidarInfoMap[lidar].lastUpdate;
}

/*
 * Get the number of rays for the given lidar
 */
int sdcSensorData::GetLidarNumRays(LidarPos lidar){
    return this->lidarInfoMap[lidar].numRays;
}

/*
 * Get the minimum angle for the given lidar
 */
sdcAngle sdcSensorData::GetLidarMinAngle(LidarPos lidar){
    return this->lidarInfoMap[lidar].minAngle;
}

/*
 * Get the angle between individual rays for the given lidar
 */
double sdcSensorData::GetLidarAngleResolution(LidarPos lidar){
    return this->lidarInfoMap[lidar].resolution;
}

/*
 * Get the range for the given lidar
 */
double sdcSensorData::GetLidarMaxRange(LidarPos lidar){
    return this->lidarInfoMap[lidar].maxRange;
}

/*
 * Retrieve a copy of the rays for the lidar in the given position
 */
std::vector<double> sdcSensorData::GetLidarRays(LidarPos lidar){
    std::vector<double> lidarRaysCopy;

    switch (lidar) {
        case FRONT:
        lidarRaysCopy = (*frontLidarRays);
        break;

        case BACK:
        lidarRaysCopy = (*backLidarRays);
        break;

        case TOP:
        lidarRaysCopy.insert(lidarRaysCopy.end(),topForwardLidarRays->begin(),topForwardLidarRays->end());
        lidarRaysCopy.insert(lidarRaysCopy.end(),topRightLidarRays->begin(),topRightLidarRays->end());
        lidarRaysCopy.insert(lidarRaysCopy.end(),topBackwardLidarRays->begin(),topBackwardLidarRays->end());
        lidarRaysCopy.insert(lidarRaysCopy.end(),topLeftLidarRays->begin(),topLeftLidarRays->end());
        break;

        case SIDE_LEFT:
        lidarRaysCopy.insert(lidarRaysCopy.end(),sideLeftBackLidarRays->begin(),sideLeftBackLidarRays->end());
        lidarRaysCopy.insert(lidarRaysCopy.end(),sideLeftFrontLidarRays->begin(),sideLeftFrontLidarRays->end());
        break;

        case SIDE_RIGHT:
        lidarRaysCopy.insert(lidarRaysCopy.end(),sideRightFrontLidarRays->begin(),sideRightFrontLidarRays->end());
        lidarRaysCopy.insert(lidarRaysCopy.end(),sideRightBackLidarRays->begin(),sideRightBackLidarRays->end());
        break;

        case TOP_FORWARD:
        lidarRaysCopy = (*topForwardLidarRays);
        break;

        case TOP_RIGHT:
        lidarRaysCopy = (*topRightLidarRays);
        break;

        case TOP_BACKWARD:
        lidarRaysCopy = (*topBackwardLidarRays);
        break;

        case TOP_LEFT:
        lidarRaysCopy = (*topLeftLidarRays);
        break;

        case SIDE_LEFT_FRONT:
        lidarRaysCopy = (*sideLeftFrontLidarRays);
        break;

        case SIDE_LEFT_BACK:
        lidarRaysCopy = (*sideLeftBackLidarRays);
        break;

        case SIDE_RIGHT_FRONT:
        lidarRaysCopy = (*sideRightFrontLidarRays);
        break;

        case SIDE_RIGHT_BACK:
        lidarRaysCopy = (*sideRightBackLidarRays);
        break;
    }

    return lidarRaysCopy;
}

/*
 * Return a vector of pairs (ray angle, ray length) which represents objects in view of front lidar
 */
std::vector<sdcLidarRay> sdcSensorData::GetBlockedFrontRays(){
    std::vector<sdcLidarRay> objectsInFront;
    for (int i = 0; i < frontLidarRays->size(); i++) {
        if (!std::isinf((*frontLidarRays)[i])) {
            sdcAngle angle = sdcAngle(lidarInfoMap[FRONT].minAngle + i*lidarInfoMap[FRONT].resolution);
            objectsInFront.push_back(sdcLidarRay(angle, (*frontLidarRays)[i]));
        }
    }
    return objectsInFront;
}

/*
 * Return a vector of pairs (ray angle, ray length) which represents objects in view of back lidar
 */
std::vector<sdcLidarRay> sdcSensorData::GetBlockedBackRays(){
    std::vector<sdcLidarRay> objectsInBack;
    for (int i = 0; i < backLidarRays->size(); i++) {
        if (!std::isinf((*backLidarRays)[i])) {
            sdcAngle angle = sdcAngle(i*backAngleResolution+backMinAngle);
            objectsInBack.push_back(sdcLidarRay(angle, (*backLidarRays)[i]));
        }
    }
    return objectsInBack;
}

/*
 * Returns a vector of objects in the front lidar constructed with their
 * left and right bounding lidar ray, as well as the minimum distance to the
 * object
 */
std::vector<sdcVisibleObject> sdcSensorData::GetObjectsInFront(){
    std::vector<sdcVisibleObject> objectList;

    // With no blocked rays, there are no objects to record
    std::vector<sdcLidarRay> blockedRays = GetBlockedFrontRays();
    if(blockedRays.size() == 0){
       // printf("no blocked rays\n");
        return objectList;
    }

    double distMargin = 1;
    double angleMargin = 0.01;

    sdcAngle objMinAngle;
    double objFirstDist;
    double objMinDist;

    sdcAngle prevAngle;
    double prevDist;
    bool ignorePrev = true;

    // Parse the blocked rays into separate objects
    for (int i = 0; i < blockedRays.size(); i++) {
        sdcAngle curAngle = blockedRays[i].angle;
        double curDist = blockedRays[i].dist;

        if(!ignorePrev){
            objMinDist = curDist < objMinDist ? curDist : objMinDist;

            // If either the checked angles or distance fall outside the margins, the rays are looking at a new object
            if(!((curAngle - prevAngle).WithinMargin(angleMargin) && fabs(curDist - prevDist) < distMargin)){
                // Record the object just found
                objectList.push_back(sdcVisibleObject(sdcLidarRay(objMinAngle, objFirstDist), sdcLidarRay(prevAngle, prevDist), objMinDist));
                ignorePrev = true;
            }
        }else{
            ignorePrev = false;
            objMinAngle = curAngle;
            objMinDist = curDist;
            objFirstDist = curDist;
        }

        prevAngle = curAngle;
        prevDist = curDist;
    }

    // Since objects are recorded on the trailing end of the loop, this will make sure the last object is properly added
    objectList.push_back(sdcVisibleObject(sdcLidarRay(objMinAngle, objFirstDist), sdcLidarRay(prevAngle, prevDist), objMinDist));
    return objectList;
}










std::vector<sdcLidarRay> sdcSensorData::GetBlockedLeftRays(){
    std::vector<sdcLidarRay> objectsOnLeft;
    for (int i = 0; i < sideLeftFrontLidarRays->size(); i++) {
        if (!std::isinf((*sideLeftFrontLidarRays)[i])) {
            sdcAngle angle = sdcAngle(lidarInfoMap[SIDE_LEFT_FRONT].minAngle + i*lidarInfoMap[SIDE_LEFT_FRONT].resolution);
            objectsOnLeft.push_back(sdcLidarRay(angle, (*sideLeftFrontLidarRays)[i]));
        }
    }
    return objectsOnLeft;
}


std::vector<sdcVisibleObject> sdcSensorData::GetObjectsOnLeft(){
    std::vector<sdcVisibleObject> objectList;

    // With no blocked rays, there are no objects to record
    std::vector<sdcLidarRay> blockedRays = GetBlockedLeftRays();
    if(blockedRays.size() == 0){
       // printf("no blocked rays\n");
        return objectList;
    }

    double distMargin = 1;
    double angleMargin = 0.01;

    sdcAngle objMinAngle;
    double objFirstDist;
    double objMinDist;

    sdcAngle prevAngle;
    double prevDist;
    bool ignorePrev = true;

    // Parse the blocked rays into separate objects
    for (int i = 0; i < blockedRays.size(); i++) {
        sdcAngle curAngle = blockedRays[i].angle;
        double curDist = blockedRays[i].dist;

        if(!ignorePrev){
            objMinDist = curDist < objMinDist ? curDist : objMinDist;

            // If either the checked angles or distance fall outside the margins, the rays are looking at a new object
            if(!((curAngle - prevAngle).WithinMargin(angleMargin) && fabs(curDist - prevDist) < distMargin)){
                // Record the object just found
                objectList.push_back(sdcVisibleObject(sdcLidarRay(objMinAngle, objFirstDist), sdcLidarRay(prevAngle, prevDist), objMinDist));
                ignorePrev = true;
            }
        }else{
            ignorePrev = false;
            objMinAngle = curAngle;
            objMinDist = curDist;
            objFirstDist = curDist;
        }

        prevAngle = curAngle;
        prevDist = curDist;
    }

    // Since objects are recorded on the trailing end of the loop, this will make sure the last object is properly added
    objectList.push_back(sdcVisibleObject(sdcLidarRay(objMinAngle, objFirstDist), sdcLidarRay(prevAngle, prevDist), objMinDist));
    return objectList;
}




std::vector<sdcLidarRay> sdcSensorData::GetBlockedRightRays(){
    std::vector<sdcLidarRay> objectsOnRight;
    for (int i = 0; i < sideRightFrontLidarRays->size(); i++) {
        if (!std::isinf((*sideRightFrontLidarRays)[i])) {
            //printf("ray not inf\n");
            sdcAngle angle = sdcAngle(lidarInfoMap[SIDE_RIGHT_FRONT].minAngle + i*lidarInfoMap[SIDE_RIGHT_FRONT].resolution);
            objectsOnRight.push_back(sdcLidarRay(angle, (*sideRightFrontLidarRays)[i]));
        }
    }
    return objectsOnRight;
}


std::vector<sdcVisibleObject> sdcSensorData::GetObjectsOnRight(){
    std::vector<sdcVisibleObject> objectList;

    // With no blocked rays, there are no objects to record
    std::vector<sdcLidarRay> blockedRays = GetBlockedRightRays();
    if(blockedRays.size() == 0){
       // printf("no blocked rays\n");
        return objectList;
    }

    double distMargin = 1;
    double angleMargin = 0.01;

    sdcAngle objMinAngle;
    double objFirstDist;
    double objMinDist;

    sdcAngle prevAngle;
    double prevDist;
    bool ignorePrev = true;

    // Parse the blocked rays into separate objects
    for (int i = 0; i < blockedRays.size(); i++) {
        sdcAngle curAngle = blockedRays[i].angle;
        double curDist = blockedRays[i].dist;

        if(!ignorePrev){
            objMinDist = curDist < objMinDist ? curDist : objMinDist;

            // If either the checked angles or distance fall outside the margins, the rays are looking at a new object
            if(!((curAngle - prevAngle).WithinMargin(angleMargin) && fabs(curDist - prevDist) < distMargin)){
                // Record the object just found
                objectList.push_back(sdcVisibleObject(sdcLidarRay(objMinAngle, objFirstDist), sdcLidarRay(prevAngle, prevDist), objMinDist));
                ignorePrev = true;
            }
        }else{
            ignorePrev = false;
            objMinAngle = curAngle;
            objMinDist = curDist;
            objFirstDist = curDist;
        }

        prevAngle = curAngle;
        prevDist = curDist;
    }

    // Since objects are recorded on the trailing end of the loop, this will make sure the last object is properly added
    objectList.push_back(sdcVisibleObject(sdcLidarRay(objMinAngle, objFirstDist), sdcLidarRay(prevAngle, prevDist), objMinDist));
    return objectList;
}



// GPS variables
/* road-driving branch
double sdcSensorData::gpsX = 0;
double sdcSensorData::gpsY = 0;
sdcAngle sdcSensorData::gpsYaw = sdcAngle(0);
*/
/*
 * Update the gps information
 */
void sdcSensorData::UpdateGPS(double x, double y, double yaw){
    gpsX = x;
    gpsY = y;
    gpsYaw = sdcAngle(yaw);
}

/*
 * Get the current sensor readings for position
 */
math::Vector2d sdcSensorData::GetPosition(){
    return math::Vector2d(gpsX, gpsY);
}

/*
 * Get the current sensor readings for orientation
 */
sdcAngle sdcSensorData::GetYaw(){
    return gpsYaw;
}
