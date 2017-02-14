/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/


/*
 * Based on UtilityCart.cc written by Nate Koenig, sdcCar provides both
 * interaction with Gazebo's simulation environment as well as logic to
 * make it behave intelligently in a variety of situations. This is the main
 * class used in the Self-Driving Comps project.
 *
 * Physics parameters and Gazebo interfacing are based on UtilityCart.
 */

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "sdcCar.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(sdcCar)

// SDC-defined constants
const double PI = 3.14159265359;

const double DIRECTION_MARGIN_OF_ERROR = 0.00855;
const double STEERING_MARGIN_OF_ERROR = 0.05;
const int LIDAR_DETECTION_MARGIN_OF_ERROR = 2;

// How fast the car turns each update
const double STEERING_ADJUSTMENT_RATE = 0.02;

// How much we can turn the "steering wheel"
const double STEERING_RANGE = 5 * PI;

const double CAR_WIDTH = 0.8;
const double CAR_LENGTH = 2.0;

// The width of the channel in front of the car for which we count objects as
// being directly in front of the car
const double FRONT_OBJECT_COLLISION_WIDTH = CAR_WIDTH + 0.5;

const sdcAngle NORTH = sdcAngle(PI/2);
const sdcAngle SOUTH = sdcAngle(3*PI/2);
const sdcAngle EAST = sdcAngle(0);
const sdcAngle WEST = sdcAngle(PI);

//destLocations
const std::vector<std::pair<double,double>> ends = {
    std::pair<double,double>(52.5, 90),
    std::pair<double,double>(90, 48), //(110,48)
    std::pair<double,double>(48, 10),
    std::pair<double,double>(10,52.5)};


const std::vector<std::pair<double,double>> intExit = {
    std::pair<double,double>(52.5, 55),
    std::pair<double,double>(55, 48),
    std::pair<double,double>(48,45),
    std::pair<double,double>(45, 52.5)
    };
//n, e, s, w
//dijkstra's stuff
std::vector<int> unvisited;

const int size = 5;
//std::pair<double,double> destination = {50,50};

//std::vector<std::pair<double,double>> destinations;


int sdcCar::carIdCount = 0;


//The Variables that define what state we are in for lane driving
bool isInStraightRoad = 1;
bool isInCurveRoad = 0;
int brakeTimes = 0;
int turnCounter = 0;
double adjustAmount = 0.0;
//double maxAdjust = 0.2;
double maxAdjust = .2;

//These are the variables for lane overtaking
int changeTurnCounter = 0;
std::vector<sdcVisibleObject> rightObjects;
int straightRoadModifier = 2000000;
bool isOvertaking = false;
bool ourCarOnRight = true;


////////////////////////////////
////////////////////////////////
// BEGIN THE BRAIN OF THE CAR //
////////////////////////////////
////////////////////////////////

/*
 * Handles all logic for driving, is called every time the car receives an update
 * request from Gazebo
 */
void sdcCar::Drive()
{


    // Possible states: stop, waypoint, intersection, follow, avoidance
    switch(this->currentState)
    {
        case laneDriving:
            this->laneDriving2017();
            printf("In lane driving\n");
        break;
        // Final state, car is finished driving
        case stop:
            //printf("in stop\n");
            //printf("targetSpeed: %f\n",this->targetSpeed);
            this->Stop();
            this->MatchTargetSpeed();
            this->MatchTargetDirection();
        break;

        // Default state; drive straight to target location
        case waypoint:

        // Handle lane driving

//          this->Accelerate();
        //  this->Stop();
            //printf("inWaypoint\n");
            this->WaypointDriving();
            this->MatchTargetSpeed();
            this->MatchTargetDirection();

        break;

        // At a stop sign, performing a turn
        case intersection:
//            printf("in intersection case\n");
//            if(this->stoppedAtSign && this->stationaryCount > 2000){
//                this->currentState = DEFAULT_STATE;
//                this->ignoreStopSignsCounter = 3000;
//            }else if(this->stoppedAtSign && this->GetSpeed() < 0.5){
//                this->stationaryCount++;
//            }else if(!this->stoppedAtSign && this->sensorData.sizeOfStopSign > 6000){
//                this->Stop();
//                this->stoppedAtSign = true;
//                this->stationaryCount = 0;
//        }

        break;

        // Follows object that is going in same direction/towards same target
        case follow:
      //      printf("following\n");
        //    this->Follow();
//         Handle lane driving
        break;


    }
}

/*
 * Handles turning based on the value of targetDirection. Calculates both which direction
 * to turn and by how much, as well as turning the actual wheel
 */
void sdcCar::MatchTargetDirection(){

    //std::cout << "match target direction" << std::endl;
    if(this->currentState == waypoint){
        sdcAngle directionAngleChange = this->GetDirection() - this->targetDirection;
        // If the car needs to turn, set the target steering amount
        if (!directionAngleChange.WithinMargin(DIRECTION_MARGIN_OF_ERROR)) {
            // The steering amount scales based on how far we have to turn, with upper and lower limits
            double proposedSteeringAmount = fmax(fmin(-this->turningLimit*tan(directionAngleChange.angle/-2), this->turningLimit), -this->turningLimit);

            // When reversing, steering directions are inverted
            if(!this->reversing){
                this->SetTargetSteeringAmount(proposedSteeringAmount);
            }else{
                this->SetTargetSteeringAmount(-proposedSteeringAmount);
            }
        }

        // Check if the car needs to steer, and apply a small turn in the corresponding direction
        if (!(std::abs(this->targetSteeringAmount - this->steeringAmount) < STEERING_MARGIN_OF_ERROR)) {
            if (this->steeringAmount < this->targetSteeringAmount) {
                this->steeringAmount = this->steeringAmount + STEERING_ADJUSTMENT_RATE;
            }else{
                this->steeringAmount = this->steeringAmount - STEERING_ADJUSTMENT_RATE;
            }
        }
    }
    else if(this->currentState == laneDriving){
        this->steeringAmount = this->cameraSensorData->GetNewSteeringMagnitude();
        //printf("sensorData id in car: %i\n", sensorData->sensorId);
    }



}

/*
 * Attempts to match the current target speed
 */
void sdcCar::MatchTargetSpeed(){
    // Invert all the values if the car should be moving backwards

    int dirConst = this->reversing ? -1 : 1;

    // If the car is moving the wrong direction or slower than the target speed, press on the gas
    if((this->reversing && this->IsMovingForwards()) || (!this->reversing && !this->IsMovingForwards()) || (this->GetSpeed() < this->targetSpeed)){
        this->gas = 1.0 * dirConst;
        this->brake = 0.0;
    } else if(this->GetSpeed() > this->targetSpeed){
        // If the car is moving faster than the target speed, brake to slow down
        this->gas = 0.0;
        if(this->reversing != this->IsMovingForwards()){
            this->brake = -2.0 * dirConst;
        } else {
            // If the car is drifting in the opposite direction it should be, don't brake
            // as this has the side effect of accelerating the car in the opposite direction
            this->brake = 0.0;
        }
    }
}

//void sdcCar::Follow() {
//    // There's nothing in front of the car, so break out of follow
//    if(this->frontObjects.size() == 0){
//        this->isTrackingObject = false;
//        this->currentState = DEFAULT_STATE;
//        return;
//    }
//
//    // The default object to follow is directly in front of the car, the max range away
////    sdcVisibleObject tracked = sdcVisibleObject(sdcLidarRay(0, sdcSensorData::GetLidarMaxRange(FRONT)), sdcLidarRay(0, sdcSensorData::GetLidarMaxRange(FRONT)), sdcSensorData::GetLidarMaxRange(FRONT));
//    sdcVisibleObject tracked;
//
//    // Already tracking an object, find it again
//    if(this->isTrackingObject){
//        bool foundTrackedObject = false;
//        for(int i = 0; i < this->frontObjects.size(); i++){
//            sdcVisibleObject obj = this->frontObjects[i];
//            if(obj.IsTracking()){
//                tracked = obj;
//                foundTrackedObject = true;
//                break;
//            }
//        }
//        if(!foundTrackedObject){
//            this->isTrackingObject = false;
//            return;
//        }
//    }else{
//        // Not tracking an object, find one that's in front of the car
//        // and start tracking it
//        for(int i = 0; i < this->frontObjects.size(); i++){
//            sdcVisibleObject obj = this->frontObjects[i];
//            if(this->IsObjectDirectlyAhead(obj)){
//                tracked = obj;
//                tracked.SetTracking(true);
//                this->isTrackingObject = true;
//                this->frontObjects[i] = tracked;
//                break;
//            }
//        }
//    }
//
//    // After the above loops, if not following anything just return
//    if(!this->isTrackingObject) return;
//
//    math::Vector2d objCenter = tracked.GetCenterPoint();
//    double objSpeed = tracked.GetEstimatedYSpeed();
//
//    // Scale our speed based on how far away the tracked object is
//    // The equation is 'scaledSpeed = (objY - 10)^3 / 2000.' which
//    // gives a scaled speed of 0 at y=10 and +-0.5 at y=20, y=0 respectively
//    double scaledSpeed = pow(objCenter.y - 10, 3) / 2000.;
//
//    // Adjust the target speed based on the speed of the object, our speed,
//    // and the above calculated scaled speed
//    double newTargetSpeed = objSpeed + this->GetSpeed() + scaledSpeed;
//    this->SetTargetSpeed(newTargetSpeed);
//
//    // If the new target speed is sufficiently low, count the car as stationary
//    if(newTargetSpeed < 0.3){
//        this->stationaryCount++;
//    }else{
//        this->stationaryCount = 0;
//    }
//
//    // If the car has been stationary for sufficiently long, stop following and start
//    // trying to navigate around the object in front of it
//    if(this->stationaryCount > 2000){
//        this->currentState = avoidance;
//        this->currentAvoidanceState = navigation;
//    }
//
//    // Set the direction of the car to be angled at the tracked object
//    if(objCenter.x != 0){
//        this->SetTargetDirection(this->GetOrientation() - sdcAngle(PI / 2.) + sdcAngle(atan2(objCenter.y, objCenter.x)));
//    }else{
//        this->SetTargetDirection(this->GetOrientation());
//    }
//}
//
/*
 * Drive from point to point in the given list
 */
void sdcCar::WaypointDriving() {
    int progress = this->waypointProgress;
    //printf("target speed: %f\n", this->targetSpeed);
    //printf("waypointvec size: %lu \n", WAYPOINT_VEC.size());

    if(progress < WAYPOINT_VEC.size()){
        // Pull the next waypoint and set the car to drive towards it
        //printf("waypointvec.size: %lu\n", WAYPOINT_VEC.size());
        //this->Accelerate();

        // Check if the car is close enough to the target to move on
        double distance = sqrt(pow(WAYPOINT_VEC[progress].pos.first - this->x,2) + pow(WAYPOINT_VEC[progress].pos.second - this->y,2));
      //  printf("distance %f", distance);

        //printf("progress: %i\n", progress);
        //printf("waypointType: %i\n", WAYPOINT_VEC[progress].waypointType);
        //printf("carId: %i, hasReservation: %i\n",carId, WAYPOINT_VEC[0].hasReservation);


        if(WAYPOINT_VEC[progress].waypointType == 3 && distance < (this->GetSpeed() * this->GetSpeed())/2.9) {
            this->currentState = stop;
            this->targetSpeed = 0;
            //printf("carId: %i stopping\n",carId);
            return;
        }
        if(progress == 2){
            float objDist = this->ObjectOnCollisionCourse();
            if(objDist < (this->GetSpeed() * this->GetSpeed())/2 + 5){
                //printf("carId: %i is far dist = %f\n", this->carId, objDist);
                //printf("carId: %i this->targetSpeed: %f\n",carId, this->targetSpeed);
                this->targetSpeed = this->targetSpeed * .9;
                return;
            }
            else{
                this->targetSpeed = fmax(this->targetSpeed * 1.01, this->maxSpeed);
            }
        }

        if (WAYPOINT_VEC[0].hasReservation == false && progress == 0){
            //printf("carId making request: %i\n", carId);
            if (distance < 20){
                auto instruction = sdcManager::reservationRequest(carId, this->x, this->y, GetSpeed(), WAYPOINT_VEC[progress].waypointType, this->destDirection, this->fromDir);
                this->targetSpeed = instruction.getSpeed();
                if (instruction.getHasReservation() == 1){
                    //printf("carId: %i, got reservation, progress: %i\n", carId, progress);
                    WAYPOINT_VEC[0].hasReservation = true;
                    //printf("carId :%i, set to true: %i\n", carId, WAYPOINT_VEC[0].hasReservation);
                }
                else{
                    float objDist = this->ObjectOnCollisionCourse();
                    if(objDist < 7 && objDist < distance){
                        //printf("carId: %i is close dist = %f\n", this->carId, objDist);
                        this->targetSpeed = .2*this->targetSpeed;
                    }
                    else{
                        this->targetSpeed = instruction.getSpeed();
                    }
                }
            }
            else {
                //Far away from the intersection
                float objDist = this->ObjectOnCollisionCourse();
                if(objDist < 7){
                    //printf("carId: %i is far dist = %f\n", this->carId, objDist);
                    this->targetSpeed = .9*this->targetSpeed;
                }
                else{
                    this->targetSpeed = this->maxSpeed;
                }
            }
        }
        else if(WAYPOINT_VEC[progress].waypointType != 3){


            //USE SPEED TO DETERMINE TURNING LIM
            if (WAYPOINT_VEC[progress].waypointType == 1) {
                //LEFT
                this->SetTurningLimit(this->GetSpeed()*5+20);
            } else if(WAYPOINT_VEC[progress].waypointType == 2) {
                //RIGHT
                this->SetTurningLimit(this->GetSpeed()*15+20);
            }
            GridTurning(WAYPOINT_VEC[progress].waypointType);
        }
        math::Vector2d nextTarget = {WAYPOINT_VEC[progress].pos.first,WAYPOINT_VEC[progress].pos.second};
        sdcAngle targetAngle = AngleToTarget(nextTarget);
        this->SetTargetDirection(targetAngle);
            // this->LanedDriving();

    } else {
        printf("should stop!\n");
        this->currentState = stop;
    }

}

/*
 * Uses camera data to detect lanes and sets targetDirection to stay as close
 * as possible to the midpoint.
 */
void sdcCar::LanedDriving() {
    int lanePos = this->cameraSensorData->LanePosition();
    this->SetTurningLimit(this->cameraSensorData->GetNewSteeringMagnitude());

    if (!(lanePos > 320 || lanePos < -320)) {
        // It's beautiful don't question it
        sdcAngle laneWeight = sdcAngle(tan(lanePos/(PI*66.19))/10);
        this->SetTargetDirection(this->GetDirection() + laneWeight);
    }
}





/*
 * Executes a turn at an intersection
 */
void sdcCar::GridTurning(int turn){
    int progress = this->waypointProgress;
   // printf("turn: %i\n", turn);
    //turn == 3 means stop
    float destX = WAYPOINT_VEC[progress].pos.first;
    float destY = WAYPOINT_VEC[progress].pos.second;
    float distance = sqrt(pow(WAYPOINT_VEC[progress].pos.first - this->x,2) + pow(WAYPOINT_VEC[progress].pos.second - this->y,2));


    //If the car has a reservation then proceed. If not stop and wait for a reservation

    //turn == 0 means go straight
     if (turn == 0){
        if(distance < 3 && WAYPOINT_VEC[0].hasReservation){
            if(progress < 2){
                this->waypointProgress++;
                if (this->waypointProgress == 2){
                    this->targetSpeed = this->maxSpeed;
                }
                printf("carId: %i, progress: %i\n", carId, this->waypointProgress);
            }
            //printf("turn == 0\n");
            //this->targetSpeed = this->maxCarSpeed;
            return;
        }
        else{

           /* if(GetSpeed() < .1){
                //printf("waypointType: %i\n", WAYPOINT_VEC[progress].waypointType);
                //printf("progress: %i\n", progress);
               // printf("waypointvec[0].x: %f\n", WAYPOINT_VEC[progress].pos.first);
               // printf("waypointvec[0].x: %f\n", WAYPOINT_VEC[progress].pos.second);

                //fflush(stdout);
                if(sdcManager::stopSignHandleRequest(carId, WAYPOINT_VEC[progress].waypointType, this->destDirection, this->fromDir)){
                    WAYPOINT_VEC[progress].hasReservation = true;
                    this->inIntersection = true;
                    printf("got reservation\n");
                    printf("destDirection: %i\n", this->destDirection);
                    fflush(stdout);
                }
            }*/
//            if(!laneStopped){
//                printf("lanestop request\n\n\n");
//                printf("fromDir: %i\n\n", this->fromDir);
//                sdcManager::laneStopRequest(this->fromDir);
//                laneStopped = true;
//            }
            //car sends reservation request when velocity is below threshold
            //sdcManager sends back response and car sets hasReservation to true
            //car needs to be able to send "out of intersection" message to sdcManager
            //this->targetSpeed = this->maxCarSpeed;
            return;
        }
    }
    //turn == 1 or 2 means turn
    else {
        if(WAYPOINT_VEC[0].hasReservation){
            if ((progress == 0 && distance < 1) || (progress == 1 && distance < 7)){
                math::Vector2d nextTarget = {WAYPOINT_VEC[progress+1].pos.first,WAYPOINT_VEC[progress+1].pos.second};
                sdcAngle targetAngle = AngleToTarget(nextTarget);
                this->SetTargetDirection(targetAngle);
                if (progress == 1){
                    printf("distance: %f, x: %f, y: %f\n", distance, WAYPOINT_VEC[progress].pos.first, WAYPOINT_VEC[progress].pos.second);
                }
                printf("turning\n");

                if(progress < 2){
                    this->waypointProgress++;
                    if (this->waypointProgress == 2){
                        this->targetSpeed = this->maxSpeed;
                    }
                    printf("progress: %i\n", this->waypointProgress);
                }
            }
        }

//        else{
//            if(GetSpeed() < .1){
//                if(sdcManager::stopSignHandleRequest(carId, WAYPOINT_VEC[progress].waypointType, this->destDirection, this->fromDir)){
//                    WAYPOINT_VEC[progress].hasReservation = true;
//                    inIntersection = true;
//                    printf("got reservation\n");
//                    printf("destDirection: %i\n", this->destDirection);
//                    fflush(stdout);
//                }
//            }
//            if(!laneStopped){
//                printf("lanestop request\n\n\n");
//                sdcManager::laneStopRequest(this->fromDir);
//                laneStopped = true;
//            }
//            this->Stop();
//            return;
//        }
    }

}





//////////////////////
// DIJKSTRA METHODS //
//////////////////////

//Generates a series of waypoints to get to the desired destination
void sdcCar::GenerateWaypoints(){
    //printf("in generateWaypoints\n");
    GetNSEW();
    initializeGraph();
    insertWaypointTypes(this->currentDir);
    WAYPOINT_VEC.push_back(intersections[2].waypoint);
    WAYPOINT_VEC.push_back(intersections[1].waypoint);
    WAYPOINT_VEC.push_back(intersections[0].waypoint);
    // for (int i = 0; i < WAYPOINT_VEC.size(); i++){
    //   printf("i: %i, x: %f, y: %f\n", i, WAYPOINT_VEC[i].pos.first, WAYPOINT_VEC[i].pos.second);
    // }

   // printf("end of genwaypoints\n");
}


//nesw
void sdcCar::initializeGraph() {
    //make the sdcIntersections
    sdcIntersection destIntersection;
    destIntersection.place = 0;
    sdcIntersection exitIntersection;
    exitIntersection.place = 1;
    sdcIntersection centerIntersection;
    centerIntersection.place = 2;
    int turnType = 0;//genRand(2); //returns if the car goes straight (0) left (1) or right (2)
    if (carId == 4 || carId == 3){
      turnType = 1;
    }
    if (carId == 2){
        turnType = 2;
    }
    //printf("turnType: %i carId: %i\n", turnType, this->carId);
    fflush(stdout);

    if(this->x > 46 && this->x < 50){ //NORTH END
      //  printf("idfirst:%i",carId);
        fflush(stdout);
        //right turns first dest is 3 past intersection
        //left turns first dest is 7 past intersection
        sdcManager::stopSignQueue(carId, 0);
        this->fromDir = 0;
        switch(turnType){
            case 0:
                destIntersection.waypoint = sdcWaypoint(0,ends[2]);
                exitIntersection.waypoint = sdcWaypoint(0,intExit[2]);
                break;
            case 1:
                destIntersection.waypoint = sdcWaypoint(0,ends[1]);
                exitIntersection.waypoint = sdcWaypoint(0,intExit[1]);
                break;
            case 2:
                destIntersection.waypoint = sdcWaypoint(0,ends[3]);
                exitIntersection.waypoint = sdcWaypoint(0,intExit[3]);
                break;
        }
        centerIntersection.waypoint = sdcWaypoint(0,std::pair<double,double>(48,55));

    }
    /*
    const std::vector<std::pair<double,double>> ends = {
        std::pair<double,double>(52.5, 90),
        std::pair<double,double>(90, 48),
        std::pair<double,double>(48, 10),
        std::pair<double,double>(10,52.5)};


    const std::vector<std::pair<double,double>> intExit = {
        std::pair<double,double>(52.5, 55),
        std::pair<double,double>(55, 48),
        std::pair<double,double>(48,45),
        std::pair<double,double>(45, 52.5)
        };
      */
    else if(this->y > 50 && this->y < 54){ //EAST
        sdcManager::stopSignQueue(carId, 1);
        this->fromDir = 1;
        switch(turnType){

            case 0:
                printf("from east carid: %i\n",carId);
                destIntersection.waypoint = sdcWaypoint(0,ends[3]);
                exitIntersection.waypoint = sdcWaypoint(0,intExit[3]);
                break;
            case 1:
                destIntersection.waypoint = sdcWaypoint(0,ends[2]);
                exitIntersection.waypoint = sdcWaypoint(0,intExit[2]);
                break;
            case 2:
                destIntersection.waypoint = sdcWaypoint(0,ends[0]);
                exitIntersection.waypoint = sdcWaypoint(0,intExit[0]);
                break;
        }
        centerIntersection.waypoint = sdcWaypoint(0,std::pair<double,double>(55,52.5));
    }

    else if(this->x > 50 && this->x < 54){ //SOUTH
        sdcManager::stopSignQueue(carId, 2);
        this->fromDir = 2;
        switch(turnType){
            case 0:
                destIntersection.waypoint = sdcWaypoint(0,ends[0]);
                exitIntersection.waypoint = sdcWaypoint(0,intExit[0]);
                break;
            case 1:
                destIntersection.waypoint = sdcWaypoint(0,ends[3]);
                exitIntersection.waypoint = sdcWaypoint(0,intExit[3]);
                break;
            case 2:
                destIntersection.waypoint = sdcWaypoint(0,ends[1]);
                exitIntersection.waypoint = sdcWaypoint(0,intExit[1]);
                break;
        }
        centerIntersection.waypoint = sdcWaypoint(0,std::pair<double,double>(52.5,45));
    }else if(this->y > 46 && this->y < 50){ //WEST
        sdcManager::stopSignQueue(carId, 3);
        this->fromDir = 3;
        switch(turnType){
            case 0:
                destIntersection.waypoint = sdcWaypoint(0,ends[1]);
                exitIntersection.waypoint = sdcWaypoint(0,intExit[1]);
                break;
            case 1:
                destIntersection.waypoint = sdcWaypoint(0,ends[0]);
                exitIntersection.waypoint = sdcWaypoint(0,intExit[0]);
                break;
            case 2:
                destIntersection.waypoint = sdcWaypoint(0,ends[2]);
                exitIntersection.waypoint = sdcWaypoint(0,intExit[2]);
                break;
        }
        centerIntersection.waypoint = sdcWaypoint(0,std::pair<double,double>(45,48));
    }
    else{
        printf("woops");
        fflush(stdout);
    }


    centerIntersection.waypoint.waypointType = turnType;
    exitIntersection.waypoint.waypointType = turnType;
    destIntersection.waypoint.waypointType = 3;
    centerIntersection.waypoint.hasReservation = false;
    //make the distance to all intersections infinity
  //  printf("centerIntersection wp type: %i\n", centerIntersection.waypoint.waypointType);
    intersections = {destIntersection, exitIntersection, centerIntersection};
    for (int i = 0; i < intersections.size(); ++i) {
        intersections[i].dist = std::numeric_limits<double>::infinity();
        intersections[i].place = i;
    }
  //  printf("end of graph\n");
}

void sdcCar::insertWaypointTypes(Direction startDir) {
    Direction curDir = startDir;
    int nextDir = intersections[2].waypoint.waypointType;
  //  printf("nextDir: %i\n", nextDir);
    int current = 1;
    int next = 0;
  // get the direction the car heads in from the current intersection to
  // the next one
    // 0 = straight, 1 = left, 2 = right, 3 = stop
    switch (curDir) {
      case north:
      //  printf("north\n");
        switch (nextDir) {
            case 0:
                this->destDirection = 0;
                break;
            case 2:
                this->destDirection = 1;
                break;
            case 1:
                this->destDirection = 3;
                break;
            case 3:
                break;

        }
        break;
    case east:
          //  printf("east\n");
            switch (nextDir) {
                case 1:
                    this->destDirection = 0;
                    break;
                case 2:
                    this->destDirection = 2;
                    break;
                case 0:
                    this->destDirection = 1;
                    break;
                case 3:
                    break;
            }
            break;
      case south:
        //printf("south");
        switch (nextDir) {
            case 0:
                this->destDirection = 2;
                break;
            case 1:
                this->destDirection = 1;
                break;
            case 2:
                this->destDirection = 3;
                break;
            case 3:
                break;
        }
        break;

      case west:
      //  printf("west");
        switch (nextDir) {
          case 2:
                this->destDirection = 0;
                break;
          case 1:
                this->destDirection = 2;
                break;
          case 0:
                this->destDirection = 3;
                break;
            case 3:
                break;
        }
        break;
    }
    //curDir = nextDir;

  //intersections[path[0]].waypoint.waypointType = WaypointType_Stop;
}


////////////////////
// HELPER METHODS //
////////////////////
int sdcCar::genRand(int max) {

//    std::default_random_engine generator;
//    std::uniform_int_distribution<int> distribution(0, max);
    int num;
    std::vector<int>randNumbs;
    for (int i = 0; i < 1000; i++){
        num = rand() % (max+1);
//        if(num == 2){
//            printf("2\n");
//        }
        randNumbs.push_back(num);
    }
    //printf("randnums: %i\n", randNumbs[carId-1]);
    fflush(stdout);
    return randNumbs[carId*100];

}
/*
 * Updates the list of objects in front of the car with the given list of new objects
 */
void sdcCar::UpdateFrontObjects(std::vector<sdcVisibleObject> newObjects){
    if(this->frontObjects.size() == 0){
        // The car wasn't tracking any objects, so just set the list equal to the new list
        this->frontObjects = newObjects;
        return;
    }

    std::vector<bool> isOldObjectMissing;
    std::vector<bool> isBrandNewObject;
    for(int i = 0; i < newObjects.size(); i++){
        isBrandNewObject.push_back(true);
    }

    // Compare each old object to the new objects, and determine
    // which of them are getting updated, which are missing, as well
    // as if any of the passed in objects are brand new
    for (int i = 0; i < this->frontObjects.size(); i++) {
        sdcVisibleObject oldObj = this->frontObjects[i];
        isOldObjectMissing.push_back(true);

        for (int j = 0; j < newObjects.size(); j++) {
            // Only match each new object to one old object
            if(!isBrandNewObject[j]) continue;
            sdcVisibleObject newObj = newObjects[j];

            if(oldObj.IsSameObject(newObj)){
                oldObj.Update(newObj);
                this->frontObjects[i] = oldObj;
                isOldObjectMissing[i] = false;
                isBrandNewObject[j] = false;
                break;
            }
        }
    }

    // Delete objects that are missing
    for(int i = isOldObjectMissing.size() - 1; i >= 0; i--){
        if(isOldObjectMissing[i]){
            this->frontObjects.erase(this->frontObjects.begin() + i);
        }
    }

    // Add brand new objects
    for(int i = 0; i < newObjects.size(); i++){
        if(isBrandNewObject[i]){
            this->frontObjects.push_back(newObjects[i]);
        }
    }
}


/*
 * Returns true if the current velocity angle matches the direction the car
 * is facing
 */
bool sdcCar::IsMovingForwards(){
    sdcAngle velAngle = GetDirection();
    sdcAngle carAngle = this->GetOrientation();
    return (carAngle - velAngle).IsFrontFacing();
}

/*
 * Gets the speed of the car
 */
double sdcCar::GetSpeed(){
    return sqrt(pow(this->velocity.x,2) + pow(this->velocity.y,2));
}

/*
 * Gets the current direction the car is travelling
 */
sdcAngle sdcCar::GetDirection(){
    math::Vector3 velocity = this->velocity;
    return sdcAngle(atan2(velocity.y, velocity.x));
}

/*
 * Gets the current direction the car is travelling in NSEW
 */
void sdcCar::GetNSEW(){
    if((this->yaw - WEST).WithinMargin(PI/4)){
        this->currentDir = west;
    } else if((this->yaw - SOUTH).WithinMargin(PI/4)){
        this->currentDir = south;
    } else if((this->yaw - EAST).WithinMargin(PI/4)){
        this->currentDir = east;
    } else {
        this->currentDir = north;
    }
}

/*
 * Gets the direction the car is facing
 */
sdcAngle sdcCar::GetOrientation(){
    return this->yaw;
}

/*
 * Returns the angle from the car's current position to a target position
 */
sdcAngle sdcCar::AngleToTarget(math::Vector2d target) {
    //math::Vector2d position = sdcSensorData::GetPosition();
    math::Vector2d targetVector = math::Vector2d(target.x - this->x, target.y - this->y);
   // printf("x vec: %f y vec: %f", target.x- this->x, target.y- this->y);
    //fflush(stdout);
    return sdcAngle(atan2(targetVector.y, targetVector.x));
}

/*
 * Returns true if there is an object ahead of the car that might collide with us if we
 * continue driving straight ahead
 */
bool sdcCar::ObjectDirectlyAhead() {
    if(this->frontObjects.size() == 0){
        return false;
    }

    for (int i = 0; i < this->frontObjects.size(); i++) {
        if(this->IsObjectDirectlyAhead(this->frontObjects[i])){
            printf("object is ahead\n");
            printf("carId: %i\n",this->carId);
            return true;
        }
    }
    return false;
}

/*
 * Returns true if the given object is directly ahead of us, else false
 */
bool sdcCar::IsObjectDirectlyAhead(sdcVisibleObject obj){
    double leftDist = obj.left.GetLateralDist();
    double rightDist = obj.right.GetLateralDist();
    if(leftDist < 0 && rightDist > 0) return true;
    return fmin(fabs(leftDist), fabs(rightDist)) < FRONT_OBJECT_COLLISION_WIDTH / 2.;
}

/*
 * Returns true if there is an object on a potential collision course with our car
 */
float sdcCar::ObjectOnCollisionCourse(){
    float minDist = 99999;
    if(this->frontObjects.size() == 0) return 99999;
    for (int i = 0; i < this->frontObjects.size(); i++) {
        /*
        if(this->IsObjectOnCollisionCourse(this->frontObjects[i])){
            return true;
        }
        */
        float dist = this->IsGoingToHit(this->frontObjects[i]);
        //printf("distance: %f\n", dist);
        if(dist > 0.0){
            minDist = fmin(dist, minDist);
        }
    }

    return minDist;
}

/*
 * Returns true if the given object is on a potential collision course with our car
 */
bool sdcCar::IsObjectOnCollisionCourse(sdcVisibleObject obj){
    bool isTooFast = this->IsObjectTooFast(obj);
    bool isTooFurious = this->IsObjectTooFurious(obj);
    return isTooFast || isTooFurious;
}

/*
 * Returns true if the given object is projected to run into the car within a short time period from now
 */
bool sdcCar::IsObjectTooFast(sdcVisibleObject obj){
    math::Vector2d centerpoint = obj.GetCenterPoint();
    bool inLineToCollide = (fabs(obj.lineIntercept) < 1.5 || (fabs(centerpoint.x) < 1.5 && fabs(obj.GetEstimatedXSpeed()) < fabs(0.1 * obj.GetEstimatedYSpeed())));
    bool willHitSoon = obj.dist / obj.GetEstimatedSpeed() < 20;
    return inLineToCollide && willHitSoon;
}

/*
 * Returns true if the given object is projected to run into the car within a short time period from now
 */
float sdcCar::IsGoingToHit(sdcVisibleObject obj){
    bool inLineToCollide = false;
    math::Vector2d centerpoint = obj.GetCenterPoint();
    double leftLateralDist = obj.GetLeft().GetLateralDist();
    double rightLateralDist = obj.GetRight().GetLateralDist();
    double midpoint = (leftLateralDist + rightLateralDist)*0.5;
      //checks if the midpoint is close enough to the car to be in the same lane
      //TO DO: we need another filter to check if it's left or right
    if (std::abs(midpoint) < 2){
          inLineToCollide = true;
    }
    if(inLineToCollide){
        return obj.dist;
    }
    else{
        //printf("not in line to collide\n");
        return -1.0;
    }
}

/*
 * Returns true if the given object is very close to the car
 */
bool sdcCar::IsObjectTooFurious(sdcVisibleObject obj){
    math::Vector2d centerpoint = obj.GetCenterPoint();
    return (fabs(centerpoint.x) < FRONT_OBJECT_COLLISION_WIDTH / 2. && fabs(centerpoint.y) < 1.5);
}

///////////////////////////
// BEGIN CONTROL METHODS //
///////////////////////////

/*
 * Speeds up the car by the given amount (in m/s) at the given rate
 *
 * Default amt: 1.0
 * Default rate: 1.0
 */
void sdcCar::Accelerate(double amt, double rate){

    this->SetTargetSpeed(this->GetSpeed() + amt);
    //printf("targetSpeed: %f\n",this->targetSpeed);
    this->SetAccelRate(rate);
    fflush(stdout);
}

/*
 * Slows down the car by the given amount (in m/s) at the given rate
 *
 * Default amt: 1.0
 * Default rate: 1.0
 */
void sdcCar::Brake(double amt, double rate){
    this->SetTargetSpeed(this->GetSpeed() - amt);
    this->SetBrakeRate(rate);
}

/*
 * Sets the target speed to 0 m/s
 */
void sdcCar::Stop(){
    this->SetTargetSpeed(0);
}

/*
 * Move the car in reverse. Target speed will now be matched with the car going
 * backwards and target direction should be the direction of velocity desired, NOT
 * the direction the front of the car is facing
 */
void sdcCar::Reverse(){
    this->reversing = true;
}

/*
 * Stop reversing the car.
 */
void sdcCar::StopReverse(){
    this->reversing = false;
}

/*

 * Sets the rate of acceleration for the car. The rate is a scalar for the
 * force applies to accelerate the car
 *
 * Default rate: 1.0, can't be negative
 */
void sdcCar::SetAccelRate(double rate){
    this->accelRate = fmax(rate, 0.0);
}

/*
 * Sets the rate of braking for the car. The rate is a scalar for the
 * force applied to brake the car
 *
 * Default rate: 1.0, can't be negative
 */
void sdcCar::SetBrakeRate(double rate){
    this->brakeRate = fmax(rate, 0.0);
}

/*
 * Sets a target direction for the car
 */
void sdcCar::SetTargetDirection(sdcAngle direction){
    this->targetDirection = direction;
}

/*
 * Sets a target steering amount for the steering wheel
 */
void sdcCar::SetTargetSteeringAmount(double a){
    this->targetSteeringAmount = a;
}

/*
 * Sets the target speed for the car, as well as resetting the brake
 * and accel rates to default. Methods wishing to change those parameters
 * should make sure to do so AFTER a call to this method
 */
void sdcCar::SetTargetSpeed(double s){
    this->targetSpeed = fmax(fmin(s, this->maxCarSpeed), 0);
    //printf("-- bool: %i--", this->stopping);
    this->stopping = (this->targetSpeed == 0);
    this->SetAccelRate();
    this->SetBrakeRate();
}

/*
 * Sets the amount by which the car turns. A larger number makes the car turn
 * harder.
 */
void sdcCar::SetTurningLimit(double limit){
    this->turningLimit = limit;
}

/*Helper methods for sdc car, SDC-Merged*/

// execute this logic when the car detects a straight road
void sdcCar::driveOnStraightRoad(double degree) {
  this->cameraSensorData->UpdateSteeringMagnitude(0);
  this->steeringAmount = this->cameraSensorData->GetNewSteeringMagnitude();
  //this->MatchTargetDirection();
  // Attempts to match the target speed
  this->MatchTargetSpeed();
}

// execute this logic when the car detects a curved road
void sdcCar::driveOnCurvedRoad(double degree) {
  int direction = degree >= 0? 1 : -1;
  this->cameraSensorData->UpdateSteeringMagnitude(1 * direction);
  this->steeringAmount = this->cameraSensorData->GetNewSteeringMagnitude();
  //this->MatchTargetDirection();
  // Attempts to match the target speed
  //this->MatchTargetSpeed();
  Brake(2,1);
}


//This is the Lane Driving portion
void sdcCar::laneDriving2017(){

  if (turnCounter > 0) {
    turnCounter++;
  }

  if (turnCounter > 3000) {
    turnCounter = 0;
  }
  double degree = this->cameraSensorData->getMidlineAngle();
  //printf("The angle is %f\n", degree);
  if(std::abs(degree) < 20){
    //printf("in straight road!\n");
    isInStraightRoad = true;
  }
  else if (std::abs(degree) >= 25) {
    //printf("in curved road!\n");
    isInCurveRoad = true;
  }

  //When its in the curve
  if (isInCurveRoad && !isInStraightRoad){
    //printf("Case 1: In Curve\n");
    //printf("adjustment to steer mag: %f\n", degree/40);
    this->cameraSensorData->UpdateSteeringMagnitude(degree/40);
    this->steeringAmount = this->cameraSensorData->GetNewSteeringMagnitude();
  }
  else if (isInStraightRoad && !isInCurveRoad){
    //printf("Case 2: In Straight\n");
    if(this->GetSpeed() < this->targetSpeed){
      //printf("Subcase: Accelerating\n");
      //printf("Get speed: %f, targed speed: %f\n", this->GetSpeed(), this->targetSpeed);
      this->gas = 1.0;
      this->brake = 0.0;
    }
    else{
      this->gas = 0;
    }
    // re-adjust angles if the difference between midline and vertical is too large
    // say >= 15 degree
    double verticalDifference = this->cameraSensorData->getVerticalDifference();
    double latestAdjustAmount;

    if (std::abs(verticalDifference) > 5 && turnCounter <= 1200) {
      //shrink abs(vertical) by 15
      double compressedVertical;
      if (verticalDifference > 0) {
        compressedVertical = verticalDifference - 10;
      } else {
        compressedVertical = verticalDifference + 10;
      }

      //square compressedVertical to react more strongly to large angles
      double squared = compressedVertical * compressedVertical;
      if (compressedVertical < 0) {
        squared = -squared;
      }

      //latestAdjustAmount is the angle difference for this turn
      latestAdjustAmount = -squared/10000;
      if (latestAdjustAmount > maxAdjust) {
        latestAdjustAmount = fmin(latestAdjustAmount, maxAdjust);
      } else {
        latestAdjustAmount = fmax(latestAdjustAmount, -maxAdjust);
      }

      //set adjustAmount, which is the average angle change at beginning
      //of counter cycle
      //limit absolute value to 0.1
      if (turnCounter == 0) {
        //printf("updated adjustamount!\n");
        adjustAmount = -squared/10000;
        if (adjustAmount > maxAdjust) {
          adjustAmount = fmin(adjustAmount, maxAdjust);
        } else {
          adjustAmount = fmax(adjustAmount, -maxAdjust);
        }
        turnCounter++;
      } else {
        adjustAmount *= turnCounter;
        adjustAmount += latestAdjustAmount;
        adjustAmount /= (turnCounter + 1);
      }

      if (turnCounter % 50 == 0) {
        //printf("ADJUST: angle %f, update %f, adjust amount %f, counter %i\n", verticalDifference, latestAdjustAmount, adjustAmount, turnCounter);
      }

      this->cameraSensorData->UpdateSteeringMagnitude(latestAdjustAmount);
      this->steeringAmount = this->cameraSensorData->GetNewSteeringMagnitude();
      //printf("turn counter: %i\n", turnCounter);
    } else if (turnCounter > 1200 && turnCounter < 1900) {
      //reverses turning angle to put the car back on track
      if (adjustAmount > 0.85*maxAdjust) {
        adjustAmount = fmin(adjustAmount, 0.85*maxAdjust);
      } else if (adjustAmount < -0.85*maxAdjust){
        adjustAmount = fmax(adjustAmount, -0.85*maxAdjust);
      }
      if(turnCounter % 50 == 0) {
        //printf("BACKTRACKING: angle is %f, updated by %f, counter %i\n", verticalDifference, -1.1*adjustAmount, turnCounter);
      }

      this->cameraSensorData->UpdateSteeringMagnitude(-1.1*adjustAmount);
      this->steeringAmount = this->cameraSensorData->GetNewSteeringMagnitude();
      //turnCounter++;
    } else {
      this->cameraSensorData->UpdateSteeringMagnitude(0);
      this->steeringAmount = this->cameraSensorData->GetNewSteeringMagnitude();
      if(turnCounter>0) {
        //printf("step 3 - reset counter\n");
      }
      turnCounter = 0;
      //printf("step 3 - reset counter\n");
    }
  }
  else{
    //if its about to exit curve and get back on straight road
    //lets try implementing an extra turn "counter" here so that we continue turning for a little bit long when we exit the curve
    if(std::abs(degree) <= 20){
      //printf("Case 3: About to exit curve\n");
      isInStraightRoad = true;
      isInCurveRoad = false;
      if(this->GetSpeed() < this->targetSpeed){
        //printf("Subcase: Accelerating\n");
        this->gas = 0.7;
        this->brake = 0.0;
      }
      double verticalDifference = this->cameraSensorData->getVerticalDifference();
      //printf("adjustment to steer mag: %f\n", degree/25);
      this->cameraSensorData->UpdateSteeringMagnitude(degree/25);
      //this->cameraSensorData->UpdateSteeringMagnitude(0);
      //printf("the angle is %f\n", verticalDifference/200);
      this->steeringAmount = this->cameraSensorData->GetNewSteeringMagnitude();
    }
    else if(std::abs(degree) >= 25){
      //printf("Case 4: About to enter curve\n");
      if (brakeTimes >= 10) {
        isInStraightRoad = false;
        isInCurveRoad = true;
        brakeTimes = 0;
      } else {
        //printf("Actually braking\n");
        Brake(4,4.5);
        //printf("adjustment to steer mag: %f\n", degree/50);
        this->cameraSensorData->UpdateSteeringMagnitude(degree/50);
        this->steeringAmount = this->cameraSensorData->GetNewSteeringMagnitude();
        brakeTimes++;
      }
    }
  }
}

/* New Code for Lane Overtaking */
void sdcCar::overtaking2017() {
  /* Overtaking decision part */
  if (this->carId == 1) {
    this->SetTargetSpeed(7);
    std::vector<sdcVisibleObject> listOfFrontObjects = this->frontObjects;
    std::vector<sdcVisibleObject> filteredListOfFrontObjects;
    std::vector<sdcVisibleObject> sameLaneObjects;
    std::vector<sdcVisibleObject> otherLaneObjects;
    std::vector<sdcVisibleObject> leftLaneObjects;
    std::vector<sdcVisibleObject> rightLaneObjects;
    //printf("the size is %i\n", listOfFrontObjects.size());
    bool carInFront = false;
    bool leftLaneFree = true;

    // First pass to filter out unreasonably small objects
    for(int j = 0; j < listOfFrontObjects.size(); j++){
      double leftLateralDist = listOfFrontObjects[j].GetLeft().GetLateralDist();
      double rightLateralDist = listOfFrontObjects[j].GetRight().GetLateralDist();
      double lateralWidth = std::abs(leftLateralDist - rightLateralDist);
      //checks whether width of object is greater than 0.2
      if (lateralWidth >= 0.2) {
        filteredListOfFrontObjects.push_back(listOfFrontObjects[j]);
      }
    }
    // Second pass to decide if the object is on the same lane or the other lane
    for(int j = 0; j < filteredListOfFrontObjects.size(); j++) {
      double leftLateralDist = filteredListOfFrontObjects[j].GetLeft().GetLateralDist();
      double rightLateralDist = filteredListOfFrontObjects[j].GetRight().GetLateralDist();
      double midpoint = (leftLateralDist + rightLateralDist)*0.5;
      //checks if the midpoint is close enough to the car to be in the same lane
      //TO DO: we need another filter to check if it's left or right
      if (std::abs(midpoint) >= 2 && std::abs(midpoint) <= 5) {
        otherLaneObjects.push_back(filteredListOfFrontObjects[j]);
      } else if (std::abs(midpoint) < 2){
        sameLaneObjects.push_back(filteredListOfFrontObjects[j]);
      }
    }
    // Partition objects into left and right lane objects based on our car's position
    ourCarOnRight = true;

    if (ourCarOnRight) {
      leftLaneObjects = otherLaneObjects;
      rightLaneObjects = sameLaneObjects;
    } else {
      leftLaneObjects = sameLaneObjects;
      rightLaneObjects = otherLaneObjects;
    }

    //printf("the size is %i\n", filteredListOfFrontObjects.size());
    if(leftLaneObjects.size() > 0) {
      leftLaneFree = false;
    }
    if(rightLaneObjects.size() > 0) {
      carInFront = true;
    }
    if(carInFront) {
      //printf("car in front!\n");
    }
    if(!leftLaneFree && !isOvertaking) {
      //printf("Our car speed: %f\n", this->GetSpeed());
      //printf("left lane blocked!\n");
    }
    if (leftLaneFree && carInFront) {
      double closestlongitude = 100;
      for(int q = 0; q < rightLaneObjects.size(); q++){
        if (rightLaneObjects[q].GetLeft().GetLongitudinalDist() < closestlongitude){
          closestlongitude = rightLaneObjects[q].GetLeft().GetLongitudinalDist();
        }
      }
      if (closestlongitude < 5){
        isOvertaking = true;
        //printf("is about to overtake\n");
      }
      //printf("yes, do the overtaking\n");
    }

    /* The logic for the car that does the Overtaking */
    if((this->GetSpeed() > this->targetSpeed - 0.1) && isOvertaking){
      //printf("enter the overtaking part\n");
      if(changeTurnCounter < 400){
        this->steeringAmount = -2;
        this->SetTargetSpeed(this->GetSpeed() + 5);
        //printf("turning left\n");
      }else if (400 <= changeTurnCounter && changeTurnCounter < 4400){
        this->steeringAmount = 0;
        //printf("going straight\n");
      }else if (4400 <= changeTurnCounter && changeTurnCounter < 4800){
        this->steeringAmount = 2;
        //printf("turning right\n");
      }
      else if (changeTurnCounter >= 4800 && changeTurnCounter <= straightRoadModifier){
        this->steeringAmount = 0;
        //printf("going straight\n");

        // use side lidar to decide when we can move back to right lane
        double leftMostLateral = 0;
        //printf("The number of right objects is %lu\n", rightObjects.size());
        for (int t = 0; t < rightObjects.size(); t++) {
          double leftLateral = rightObjects[t].GetLeft().GetLateralDist();
          if(leftLateral <= leftMostLateral) {
              leftMostLateral = leftLateral;
          }
        }

        if (leftMostLateral >= 0) {
          straightRoadModifier = changeTurnCounter - 1;
        }
        //printf("The leftmost lateral is %f\n", leftMostLateral);
      } else if(changeTurnCounter > straightRoadModifier && changeTurnCounter <= straightRoadModifier + 400){
        this->steeringAmount = 2;
        //printf("turning right\n");
      }else if (straightRoadModifier + 400 <= changeTurnCounter && changeTurnCounter < straightRoadModifier + 4400){
        this->steeringAmount = 0;
        //printf("going straight\n");
      }else if (straightRoadModifier + 4400 <= changeTurnCounter && changeTurnCounter < straightRoadModifier + 4800){
        this->steeringAmount = -2;
        //printf("turning left\n");
      }else if (changeTurnCounter >= straightRoadModifier + 4800 && changeTurnCounter < straightRoadModifier + 6000) {
        this->steeringAmount = 0;
        //printf(" going straight\n");
      } else if (changeTurnCounter >= straightRoadModifier + 6000 && changeTurnCounter <= straightRoadModifier + 6001) {
        isOvertaking = false;
        straightRoadModifier = 2000000;
        changeTurnCounter = -1;
      }
      changeTurnCounter++;
    }


  }
  //printf("The car id is %i and the front lidar id is%i\n", this->carId, this->frontLidar->GetId() + 8);
  //this->MatchTargetDirection();
  // Attempts to match the target speed
  this->MatchTargetSpeed();
}

//////////////////////////////////////////////////////////////
// GAZEBO METHODS - GAZEBO CALLS THESE AT APPROPRIATE TIMES //
//////////////////////////////////////////////////////////////
void sdcCar::OnUpdate()
{
    //updates the rate in sdcManager incase the sim time is slower than real time

    if(carId == 1){
        //std::chrono::milliseconds currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        //int diffTime = currentTime.count() - msStartTime.count();
        //float tm = diffTime / float(1000);

        if (!this->setRate){
            common::Time curSimTime = this->world->GetSimTime();
            common::Time curRealTime = this->world->GetRealTime();
            float curSimTm = curSimTime.Float();
            float curRealTm = curRealTime.Float();
            //printf("curRealTm:%f\n", curRealTm);
            //float startTm = this->simStartTime.Float();
            //float simTm = curSimTm - startTm;
            //float realTm = curRealTm - startTm;
            //printf("diffTime: %f\n", diffSimTime);
            float runRate = curSimTm / curRealTm;
            //printf("runRate: %f\n", runRate);
            if((runRate < .95) && (curRealTm > .5)){
                sdcManager::setRate(runRate);
                this->setRate = true;
            }
        }
    }

    //printf("\nin onupdate\n");
    //    if(this->getSensor){
    //
    //        this->getSensor = false;
    //    }
    //this->sensorData = sdcManager::getSensorData(carId);
    // if(this->carId == 1){
    //     printf("current target speed: %f\n", this->targetSpeed);
    // }


    //REMEMBER TO CHANGE THIS
    int crudeSwitch = 2; //in merged world use 0
    //in lanedriving use 1
    //in intersection world use 2
    if(this->currentState != stop){
        if (crudeSwitch == 0) {
            if((this->x >= 0 && this->x <= 100) && (this->y >= 0 && this->y <= 100)){
                this->currentState = waypoint;
            }else{
                printf("in lanedriving crude switch\n");
                this->currentState = laneDriving;
            }
        } else if (crudeSwitch == 1){
            if((this->x <= 10 && this->x >= -10) && (this->y <= 10 && this->y >= -10)){
                //printf("starting at %f,%f \n", this->x, this->y);
                this->currentState = laneDriving;
            }
            this->currentState = laneDriving;
        } else if (crudeSwitch == 2) {
            this->currentState = waypoint;
            //printf("Crude switch == 2 : for intersection worlds");
        }
    }
    if (crudeSwitch == 0 && !((this->x <= 10 && this->x >= -10) && (this->y <= 10 && this->y >= -10))) {
        this->currentState = laneDriving;
    }


    //    if(this->stopping){
    //        printf("stopping\n");
    //        printf("%f",this->velocity.y);
    //    }
    //    if(this->turning == true){
    //        printf("speed: %f \n",this->GetSpeed());
    //    }
    //
    //     Get the current velocity of the car

    ///For stop sign algorithm queue to correctly stop
//    if(this->currentState!=laneDriving){
//        if(sdcManager::shouldStop(carId, fromDir)){
//            if(!laneStopped){
//                //printf("told to stop\n");
//                this->currentState = stop;
//                this->toldToStop = true;
//            }
//        }
//        else{
//            if(this->toldToStop){
//                this->currentState = waypoint;
//                this->toldToStop = false;
//            }
//        }
//    }


    if(this->inIntersection){
        //if outside intersection
        //printf("direction: %i\n", this->destDirection);
        // fflush(stdout);
        switch (this->destDirection) {
                //0 = north, 1 = east, 2 = south, 3 = west
            case 0:
                if(this->y > 55){
                    sdcManager::stopSignCarLeft(this->carId);
                    this->inIntersection = false;
                    // this->currentState == waypoint;
                    printf("exited north\n");
                }
                break;
            case 1:
                if(this->x  > 55){
                    sdcManager::stopSignCarLeft(this->carId);
                    this->inIntersection = false;
                    printf("exited east\n");
                }
                break;
            case 2:
                if(this->y < 45){
                    sdcManager::stopSignCarLeft(this->carId);
                    this->inIntersection = false;
                    printf("exited south\n");
                }
                break;
            case 3:
                if(this->x < 45 ){
                    sdcManager::stopSignCarLeft(this->carId);
                    this->inIntersection = false;
                    printf("exited west\n");
                }
                break;
        }
    }
    this->velocity = this->chassis->GetWorldLinearVel();
    // Get the cars current position
    math::Pose pose = this->chassis->GetWorldPose();
    this->yaw = sdcAngle(pose.rot.GetYaw());
    this->x = pose.pos.x;
    this->y = pose.pos.y;
    // this->x = pose.x;
    // this->y = pose.y;
    //printf("x, y is : %f %f \n", this->x, this->y);
    // Get the cars current rotation
    // this->yaw = sdcSensorData::GetYaw();

    // Check if the front lidars have been updated, and if they have update
    // the car's list
    // printf("\nin onupdate\n");
    if(this->frontLidarLastUpdate != this->lidarSensorData->GetLidarLastUpdate(FRONT)){
    //        printf("updating front objects\n");
      std::vector<sdcVisibleObject> v = this->lidarSensorData->GetObjectsInFront();
    //        //printf("visibleobjects size: %lu\n", v.size());
      fflush(stdout);
      this->UpdateFrontObjects(v);
      this->frontLidarLastUpdate = this->lidarSensorData->GetLidarLastUpdate(FRONT);
    }
    //  printf("\nafter lidar\n");

    // Call our Drive function, which is the brain for the car
    //printf("going to drive\n");
    ///std::cout << "on update the car model" << std::endl;
    // Get the current velocity of the car
    /*road-driving branch
     this->velocity = this->chassis->GetWorldLinearVel();
     // Get the cars current position
     math::Vector2d pose = sdcSensorData::GetPosition();
     this->x = pose.x;
     this->y = pose.y;
     // Get the cars current rotation
     this->yaw = sdcSensorData::GetYaw();
     */

     //printf("Calling getobjectsonright\n");
     std::vector<sdcVisibleObject> v = this->lidarSensorData->GetObjectsInFront();
     rightObjects = this->rightLidarSensorData->GetObjectsOnRight();
     //printf("here\n");
     //if(v[0] != NULL){printf("objects in front: %f\n", v[0].GetEstimatedSpeed());}
     //else{printf("no objects in front\n");}

    /* if(v.size() > 0){
       printf("object 0 left lateral: %f\n", v[0].GetLeft().GetLateralDist());
       printf("object 0 left longitudinal: %f\n", v[0].GetLeft().GetLongitudinalDist());
       printf("object 0 right lateral: %f\n", v[0].GetRight().GetLateralDist());
       printf("object 0 right longitudinal: %f\n", v[0].GetRight().GetLongitudinalDist());
       printf("object 0 dist: %f\n", v[0].GetDist());
     } else {
       //printf("no objects detected\n");
     }*/
     if(rightObjects.size() > 0){
       //printf("object 0 left lateral: %f\n", rightObjects[0].GetLeft().GetLateralDist());
       //printf("object 0 left longitudinal: %f\n", rightObjects[0].GetLeft().GetLongitudinalDist());
       //printf("object 0 right lateral: %f\n", rightObjects[0].GetRight().GetLateralDist());
       //printf("object 0 right longitudinal: %f\n", rightObjects[0].GetRight().GetLongitudinalDist());
       //printf("object 0 dist: %f\n", rightObjects[0].GetDist());
     } else {
       //printf("no objects detected\n");
     }
    // Call our Drive function, which is the brain for the car
    //if(this->carId == 1){
        this->Drive();
    //}
    //else{
        //this->Stop();
    //}

    //printf("after drive\n");

    ////////////////////////////
    // GAZEBO PHYSICS METHODS //
    ////////////////////////////

    // Compute the angle of the front wheels.
    double wheelAngle = this->steeringAmount / this->steeringRatio;

    // Compute the rotational velocity of the wheels
    double jointVel = (this->gas-this->brake * this->maxSpeed) / this->wheelRadius;

    // Set velocity and max force for each wheel
    this->joints[0]->SetVelocityLimit(1, -jointVel);
    this->joints[0]->SetForce(1, -(this->gas * this->accelRate + this->brake * this->brakeRate) * this->frontPower);

    this->joints[1]->SetVelocityLimit(1, -jointVel);
    this->joints[1]->SetForce(1, -(this->gas * this->accelRate + this->brake * this->brakeRate) * this->frontPower);

    this->joints[2]->SetVelocityLimit(1, -jointVel);
    this->joints[2]->SetForce(1, -(this->gas * this->accelRate + this->brake * this->brakeRate) * this->rearPower);

    this->joints[3]->SetVelocityLimit(1, -jointVel);
    this->joints[3]->SetForce(1, -(this->gas * this->accelRate + this->brake * this->brakeRate) * this->rearPower);

    // Set the front-left wheel angle
    this->joints[0]->SetLowStop(0, wheelAngle);
    this->joints[0]->SetHighStop(0, wheelAngle);
    this->joints[0]->SetLowStop(0, wheelAngle);
    this->joints[0]->SetHighStop(0, wheelAngle);

    // Set the front-right wheel angle
    this->joints[1]->SetHighStop(0, wheelAngle);
    this->joints[1]->SetLowStop(0, wheelAngle);
    this->joints[1]->SetHighStop(0, wheelAngle);
    this->joints[1]->SetLowStop(0, wheelAngle);

    //  aerodynamics
    this->chassis->AddForce(math::Vector3(0, 0, this->aeroLoad * this->velocity.GetSquaredLength()));

    // Sway bars
    math::Vector3 bodyPoint;
    math::Vector3 hingePoint;
    math::Vector3 axis;

    // Physics calculations
    for (int ix = 0; ix < 4; ++ix)
    {
        hingePoint = this->joints[ix]->GetAnchor(0);
        bodyPoint = this->joints[ix]->GetAnchor(1);

        axis = this->joints[ix]->GetGlobalAxis(0).Round();
        double displacement = (bodyPoint - hingePoint).Dot(axis);

        float amt = displacement * this->swayForce;
        if (displacement > 0)
        {
            if (amt > 15)
                amt = 15;

            math::Pose p = this->joints[ix]->GetChild()->GetWorldPose();
            this->joints[ix]->GetChild()->AddForce(axis * -amt);
            this->chassis->AddForceAtWorldPosition(axis * amt, p.pos);

            p = this->joints[ix^1]->GetChild()->GetWorldPose();
            this->joints[ix^1]->GetChild()->AddForce(axis * amt);
            this->chassis->AddForceAtWorldPosition(axis * -amt, p.pos);
        }
    }
    //    printf("current state: %u\n",this->currentState);
}


/*
 * Called when initially loading the car model from the sdf. Links the car
 * to the OnUpdate methods so we can receive updates
 */


void sdcCar::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Store the model and chassis of the car for later access
    this->model = _model;
    this->world = _model->GetWorld();
    if(carId == 1){
        //this->msStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        this->simStartTime = this->world->GetStartTime();
        this->setRate = false;
    }
    this->chassis = this->model->GetLink(_sdf->Get<std::string>("chassis"));
    this->camera = this->model->GetLink(_sdf->Get<std::string>("camera"));
    this->frontLidar = this->model->GetLink(_sdf->Get<std::string>("frontLidar"));
    this->frontLidarId = this->frontLidar->GetId() + 8;
    //printf("lidar Id: %i\n", this->frontLidarId);
    this->cameraId = this->camera->GetId() + 2;
    //printf("---camera ID: %i---\n",this->cameraId);
    this->leftSideLidar = this->model->GetLink(_sdf->Get<std::string>("leftSideLidar"));
    this->leftLidarId = this->leftSideLidar->GetId() + 8;
    //printf("left id: %i\n", this->leftLidarId);

    this->rightSideLidar = this->model->GetLink(_sdf->Get<std::string>("rightSideLidar"));
    this->rightLidarId = this->rightSideLidar->GetId() + 8;
    //printf("right id: %i\n", this->rightLidarId);
    // Get all the wheel joints
    this->joints[0] = this->model->GetJoint(_sdf->Get<std::string>("front_left"));
    this->joints[1] = this->model->GetJoint(_sdf->Get<std::string>("front_right"));
    this->joints[2] = this->model->GetJoint(_sdf->Get<std::string>("back_left"));
    this->joints[3] = this->model->GetJoint(_sdf->Get<std::string>("back_right"));

    // Pull some parameters that are defined in the sdf
    this->maxSpeed = _sdf->Get<double>("max_speed");
    this->aeroLoad = _sdf->Get<double>("aero_load");
    this->tireAngleRange = _sdf->Get<double>("tire_angle_range");
    this->frontPower = _sdf->Get<double>("front_power");
    this->rearPower = _sdf->Get<double>("rear_power");
    this->wheelRadius = _sdf->Get<double>("wheel_radius");

    //this->frontSensor = this->model->GetLink(_sdf->Get<std::string>("front_lidar"));

    // Tell Gazebo to call OnUpdate whenever the car needs an update
    this->connections.push_back(event::Events::ConnectWorldUpdateBegin(boost::bind(&sdcCar::OnUpdate, this)));
}

/*
 * Called when the car and world are being (re)initialized.
 */
void sdcCar::Init()
{
    //printf("started init");
//    destinations[0] = {48,10};
//    destinations[1] = {90,48};
   // printf("created destinations");
    // Compute the angle ratio between the steering wheel and the tires
    this->steeringRatio = STEERING_RANGE / this->tireAngleRange;
    this->laneStopped = false;
    this->cameraSensorData = manager::getSensorData(cameraId);
    this->lidarSensorData = manager::getSensorData(frontLidarId);
    this->leftLidarSensorData = manager::getSensorData(this->leftLidarId);
    this->rightLidarSensorData = manager::getSensorData(this->rightLidarId);
    // During init, sensors aren't available so pull position and rotation information
    // straight from the car
    math::Pose pose = this->chassis->GetWorldPose();
    this->yaw = sdcAngle(pose.rot.GetYaw());
    this->x = pose.pos.x;
    this->y = pose.pos.y;
    //sdcSensorData::sdcSensorData();
    this->toldToStop = false;

    time_t seconds;
    srand ((unsigned)time(&seconds));
    //this->sensorData.InitLidar(LidarPos lidar, double minAngle, double angleResolution, double maxRange, int numRays);
    //sdcFrontLidarSensor::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/);
    GenerateWaypoints();

}

/*
 * Called whenever Gazebo needs an update for this model
 */
/*
 * Constructor for the car. Sets several parameters to default values, some of
 * which will get overwritten in Load or Init and others that will be updated
 * when the car is updating
 */
sdcCar::sdcCar(){
    if (this->carIdCount == 0) {
      sdcManager::sdcManager(0);
    }
    //sdcManager::registerCar(carId++);
    this->carIdCount ++;
    this->carId = this->carIdCount;
    this->inIntersection = false;
    this->destDirection = -1;
    //printf("sdcCar Steer Mag: %f\n",this->sensorData.GetNewSteeringMagnitude());
    //printf("sdcCar sensorId: %i\n",this->sensorData->sensorId);
    //this->frontSensor = sdcFrontLidarSensor();

    this->joints.resize(4);

    // Physics variables
    this->aeroLoad = 0.1;
    this->swayForce = 10;

    this->maxSpeed = 6;
    this->maxTurnLeft = 5;
    this->maxTurnRight = 5;
    this->frontPower = 50;
    this->rearPower = 50;
    this->wheelRadius = 0.3;
    this->steeringRatio = 1.0;
    this->tireAngleRange = 1.0;

    // Movement parameters
    this->gas = 0.0;
    this->brake = 0.0;
    this->accelRate = 1.0;
    this->brakeRate = 1.0;

    // Limits on the car's speed
    this->maxCarSpeed = 10;
    this->maxCarReverseSpeed = -10;

    // Initialize state enums
    this->DEFAULT_STATE = waypoint;
    this->currentState = DEFAULT_STATE;

    this->currentPerpendicularState = backPark;
    this->currentParallelState = rightBack;
    this->currentAvoidanceState = notAvoiding;

    // Set starting speed parameters
    this->targetSpeed = this->maxCarSpeed;

    // Set starting turning parameters
    this->steeringAmount = 0.0;
    this->targetSteeringAmount = 0.0;
    this->targetDirection = sdcAngle(0);
    this->turningLimit = 20.0;

    // Booleans for the car's actions
    this->turning = false;
    this->reversing = false;
    this->stopping = false;

    // Variables for parking
    this->targetParkingAngle = sdcAngle(0.0);
    this->parkingAngleSet = false;
    this->isFixingParking = false;
    this->parkingSpotSet = false;

    // Variables for waypoint driving
    this->waypointProgress = 0;

    // Variables for intersections
    this->stoppedAtSign = false;
    this->ignoreStopSignsCounter = 0;
    this->atIntersection = 0;

    // Variables for following
    this->isTrackingObject = false;
    this->stationaryCount = 0;

    // Variables for avoidance
    this->trackingNavWaypoint = false;
}
