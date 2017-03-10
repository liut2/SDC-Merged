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
#include <limits>

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

// constants used for lane centering and passing
const double MAX_ADJUST = .4;
const double MIN_DIST_TO_PASS = 15;

// The width of the channel in front of the car for which we count objects as
// being directly in front of the car
const double FRONT_OBJECT_COLLISION_WIDTH = CAR_WIDTH + 0.5;

const sdcAngle NORTH = sdcAngle(PI/2);
const sdcAngle SOUTH = sdcAngle(3*PI/2);
const sdcAngle EAST = sdcAngle(0);
const sdcAngle WEST = sdcAngle(PI);

//resetPoses
const std::vector<math::Pose> resetPose_Vec = {
    math::Pose(48,80,.001,0,0,-1.56), //N
    math::Pose(80,52.5,.001,0,0,3.1415), //E
    math::Pose(52.5,20,.001,0,0,1.56), //S
    math::Pose(20,48,.001,0,0,0), //W
    //math::Pose(525, -102.5, .1, 0, 0, 0),// dummy car 1
    math::Pose(437, -80.5, .0781, 0, 0, 0),// dummy car 1
    //math::Pose(535, -102.5, 0.1, 0, 0, 0)// dummy car 2
    math::Pose(447, -80.5, .0781, 0, 0, 0),// dummy car 1
    //combined world proposed
    math::Pose(48,80,.031,0,0,-1.56), //N
    math::Pose(80,52.5,.031,0,0,3.1415), //E
    math::Pose(52.5,20,.031,0,0,1.56), //S
    math::Pose(20,48,.031,0,0,0), //W
    }; //W

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


int sdcCar::carIdCount = 0;
int sdcCar::numCarPass = 0;
float sdcCar::carsPerMinute = 0;
bool sdcCar::teleport = false;
std::vector<int> sdcCar::resetClear(10,1);



int turnType = 0;
//intersection testing vars
int sdcCar::carAmountRight = 0;
int sdcCar::carAmountLeft = 0;
int sdcCar::carAmountStraight = 0;
float sdcCar::totalRightTimeAmount = 0;
float sdcCar::totalLeftTimeAmount = 0;
float sdcCar::totalStraightTimeAmount = 0;
bool sdcCar::twoMinCheck = false;
float carStartTime = 0;

//The Variables that define what state we are in for lane driving
bool isInStraightRoad = 1;
bool isInCurveRoad = 0;
int brakeTimes = 0;
int turnCounter = 0;
int curvedTurnCounter = 0;
int nonCurvedTurnCounter = 0;
double averageDegree = 0.0;

double adjustAmount = 0.0;


//These are the variables for lane overtaking
int changeTurnCounter = 0;
std::vector<sdcVisibleObject> rightObjects;
int straightRoadModifier = std::numeric_limits<int>::max();


bool isOvertaking = false;
bool ourCarOnRight = true;
bool isSideClearOfCars = false;
float overtakeTimeElapsed = 0;
double overTakeDistTravelled = 0;
float initialSimTime = 0;
bool otherCarsStart = false;

double previousAngle = 0;

//not using these as of now - overtaking2017_new
bool changingLanes = false;
bool hasMovedEnough = true;



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
          combinedDriving2017();


        break;
        // Final state, car is finished driving
        case stop:
            this->Stop();
            this->MatchTargetSpeed();
            this->MatchTargetDirection();
            // Teleporting for reservation cars. Once they reach their final destination and stop we reset them to a random
            // starting location and they go through the intersection again.
            if(this->crudeSwitch == 3){
                if(this->waypointProgress < STOP_SIGN_WAYPOINT_VEC.size()){
                    break;
                }
            }
            if(this->GetSpeed() < .001){
                // reset the cars pose using model->SetLinkWorldPose
                if (!this->hasReset){
                    int randIndex = genRand(3); //get a random index from 0 to 3
                    if(this->combined){
                        randIndex += 6; //Because the combined world road height is different
                    }
                    if(this->teleport){
                        if(carId == 2){
                            randIndex = 4;
                        }
                        else if(carId == 3){
                            randIndex = 5;
                        }
                    }
                    this->resetPose = resetPose_Vec[randIndex];
                    if(this->teleport && (carId == 2 || carId == 3)){
                        this->model->SetLinkWorldPose(this->resetPose,this->chassis);
                        this->currentState = laneDriving;
                        this->targetSpeed = 0;
                    }
                    else{
                        //Check to see if a car was recently teleported to that location
                        //If not do not reset the car
                        if (this->resetClear[randIndex] == 1){
                            this->model->SetLinkWorldPose(this->resetPose,this->chassis);
                            this->resetClear[randIndex] = 0;
                            this->clearIndex = randIndex;
                            this->hasReset = true;
                        }
                    }
                }
                //Re initialize the car so that it gets new random waypoints.
                else{
                    this->Init();
                }

            }
        break;

        // Default state; drive straight to target location
        case waypoint:

        // Handle lane driving

            if(this->crudeSwitch == 3){
                this->StopSignWaypointDriving();
            }
            else{
                this->WaypointDriving();
            }
            this->MatchTargetSpeed();
            this->MatchTargetDirection();

        break;

        // At a stop sign, performing a turn
        case intersection:


        break;

        // Follows object that is going in same direction/towards same target
        case follow:

        break;


    }
}

/*
 * Handles turning based on the value of targetDirection. Calculates both which direction
 * to turn and by how much, as well as turning the actual wheel
 */
void sdcCar::MatchTargetDirection(){
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
        if(carId == 1){
            this->steeringAmount = this->cameraSensorData->GetNewSteeringMagnitude();
        }
        else{
          this->steeringAmount = 0;
        }
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


/*
 * Drive from point to point in the given list
 */
void sdcCar::WaypointDriving() {
    int progress = this->waypointProgress;
    if(progress < WAYPOINT_VEC.size()){
        // Pull the next waypoint and set the car to drive towards it
        // Check if the car is close enough to the target to move on
        double distance = sqrt(pow(WAYPOINT_VEC[progress].pos.first - this->x,2) + pow(WAYPOINT_VEC[progress].pos.second - this->y,2));
        if(WAYPOINT_VEC[progress].waypointType == 3 && distance < (this->GetSpeed() * this->GetSpeed())/2.9) {
            this->currentState = stop;
            this->targetSpeed = 0;
            return;
        }
        if(progress == 2){
            float objDist = this->ObjectOnCollisionCourse();
            if(objDist < (this->GetSpeed() * this->GetSpeed())/2 + 2){
                this->targetSpeed = this->targetSpeed * .9;
                return;
            }
            else{
                this->targetSpeed = fmax(this->targetSpeed * 1.01, this->maxSpeed);
            }
        }

        if (WAYPOINT_VEC[0].hasReservation == false && progress == 0){
            if (distance < 20){
                if (this->ObjectOnCollisionCourse() > distance - 3) {
                    auto instruction = sdcManager::reservationRequest(carId, this->x, this->y, GetSpeed(), WAYPOINT_VEC[progress].waypointType, this->destDirection, this->fromDir);
                    if (instruction.getHasReservation() == 1){
                        this->targetSpeed = instruction.getSpeed();
                        WAYPOINT_VEC[0].hasReservation = true;
                    }
                    else{
                        float objDist = this->ObjectOnCollisionCourse();
                        if(objDist < 7 && objDist < distance){
                            this->targetSpeed = .2*this->targetSpeed;
                        }
                        else{
                            if (distance < 5){
                                this->targetSpeed = instruction.getSpeed();
                            }
                            else{
                                this->targetSpeed = fmax(instruction.getSpeed(),1);
                            }
                        }
                    }
                }
                else {
                  this->targetSpeed = .9*this->targetSpeed;
                }

            }
            else {
                //Far away from the intersection
                float objDist = this->ObjectOnCollisionCourse();
                if(objDist < 7){
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

    } else {
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
        sdcAngle laneWeight = sdcAngle(tan(lanePos/(PI*66.19))/10);
        this->SetTargetDirection(this->GetDirection() + laneWeight);
    }
}


/*
 * Executes a turn at an intersection
 */
void sdcCar::GridTurning(int turn){
    int progress = this->waypointProgress;
    //turn == 3 means stop
    float destX = WAYPOINT_VEC[progress].pos.first;
    float destY = WAYPOINT_VEC[progress].pos.second;
    float distance = sqrt(pow(WAYPOINT_VEC[progress].pos.first - this->x,2) + pow(WAYPOINT_VEC[progress].pos.second - this->y,2));

    //If the car has a reservation then proceed. If not stop and wait for a reservation
     if (turn == 0){
        if(distance < 3 && WAYPOINT_VEC[0].hasReservation){
            if(progress < 2){
                this->waypointProgress++;
                if (this->waypointProgress == 2){
                    this->targetSpeed = this->maxSpeed;
                }
            }
            return;
        }
        else{
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
                    //printf("distance: %f, x: %f, y: %f\n", distance, WAYPOINT_VEC[progress].pos.first, WAYPOINT_VEC[progress].pos.second);
                }
                if(progress < 2){
                    this->waypointProgress++;
                    if (this->waypointProgress == 2){
                        this->targetSpeed = this->maxSpeed;
                    }
                    //printf("progress: %i\n", this->waypointProgress);
                }
            }
        }
    }

}

//////////////////////
// DIJKSTRA METHODS //
//////////////////////

//Generates a series of waypoints to get to the desired destination
void sdcCar::GenerateWaypoints(){
    GetNSEW();
    if (this->crudeSwitch == 3) {
        StopSignInitializeGraph();
        insertWaypointTypes(this->currentDir);
        STOP_SIGN_WAYPOINT_VEC.push_back(intersections[1].waypoint);
        STOP_SIGN_WAYPOINT_VEC.push_back(intersections[0].waypoint);
    } else {
        initializeGraph();
        insertWaypointTypes(this->currentDir);
        WAYPOINT_VEC.push_back(intersections[2].waypoint);
        WAYPOINT_VEC.push_back(intersections[1].waypoint);
        WAYPOINT_VEC.push_back(intersections[0].waypoint);
    }
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
    turnType = genRand(2); //returns if the car goes straight (0) left (1) or right (2)
    if(this->crudeSwitch == 0){
        if (carId == 1){
          turnType = 0;
        }
    }
    fflush(stdout);

    if(this->x > 46 && this->x < 50){ //NORTH END
        fflush(stdout);
        //right turns first dest is 3 past intersection
        //left turns first dest is 7 past intersection
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
    else if(this->y > 50 && this->y < 54){ //EAST
        this->fromDir = 1;
        switch(turnType){

            case 0:
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
        this->fromDir = 3;
        switch(turnType){
            case 0:
                if(this->crudeSwitch == 0){
                    if(carId == 1){
                        destIntersection.waypoint = sdcWaypoint(0,std::pair<double,double>(110, 48));
                    }
                    else{
                        destIntersection.waypoint = sdcWaypoint(0,ends[1]);
                    }
                }
                else{
                    destIntersection.waypoint = sdcWaypoint(0,ends[1]);
                }
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
        fflush(stdout);
    }


    centerIntersection.waypoint.waypointType = turnType;
    exitIntersection.waypoint.waypointType = turnType;
    destIntersection.waypoint.waypointType = 3;
    if(this->crudeSwitch == 0){
        if (carId == 1){
            destIntersection.waypoint.waypointType = 0;
        }
    }
    centerIntersection.waypoint.hasReservation = false;
    //make the distance to all intersections infinity
    intersections = {destIntersection, exitIntersection, centerIntersection};
    for (int i = 0; i < intersections.size(); ++i) {
        intersections[i].dist = std::numeric_limits<double>::infinity();
        intersections[i].place = i;
    }
}

void sdcCar::insertWaypointTypes(Direction startDir) {
    Direction curDir = startDir;
    int nextDir = 0;
    if(this->crudeSwitch == 3){
        nextDir = intersections[1].waypoint.waypointType;
    }
    else{
        nextDir = intersections[2].waypoint.waypointType;
    }
    int current = 1;
    int next = 0;
  // get the direction the car heads in from the current intersection to
  // the next one
    // 0 = straight, 1 = left, 2 = right, 3 = stop
    switch (curDir) {
      case north:
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
}


////////////////////
// HELPER METHODS //
////////////////////
int sdcCar::genRand(int max) {
    int num;
    std::vector<int>randNumbs;
    for (int i = 0; i < 1000; i++){
        num = rand() % (max+1);
        randNumbs.push_back(num);
    }
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
    math::Vector2d targetVector = math::Vector2d(target.x - this->x, target.y - this->y);
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
        float dist = this->IsGoingToHit(this->frontObjects[i]);
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
    if (std::abs(midpoint) < 2){
          inLineToCollide = true;
    }
    if(inLineToCollide){
        return obj.dist;
    }
    else{
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
  // Attempts to match the target speed
  this->MatchTargetSpeed();
}

// execute this logic when the car detects a curved road
void sdcCar::driveOnCurvedRoad(double degree) {
  int direction = degree >= 0? 1 : -1;
  this->cameraSensorData->UpdateSteeringMagnitude(1 * direction);
  this->steeringAmount = this->cameraSensorData->GetNewSteeringMagnitude();
  // Attempts to match the target speed
  Brake(2,1);
}

// Combine the lane driving and lane overtaking
void sdcCar::combinedDriving2017() {
  if(this->carId == 1){

    this->SetTargetSpeed(7);
    if (!this->obstacleInFront) {
      MatchTargetSpeed();
    } else {
      this->gas = 0.0;
      this->brake = -1.0;
    }
    float switchDistance = 0;
    float startDistance = 0;
    if (this->combined){
        switchDistance = 436.65;
        startDistance = 362.8;
    }
    else{
        switchDistance = 257;
        startDistance = 257-30;
    }
    if (this->x < switchDistance){
      this->laneDriving2017();
      if (this->x >= startDistance) {
        otherCarsStart = true;
      }
    }

    else if (this->x >= switchDistance) {
      if (!isOvertaking) {
        this->laneDriving2017();
      } else {
        this->overtaking2017();
      }

      if (!isOvertaking) {
        this->obstacleInFront = false;
        isOvertaking = shouldWeOvertake();
        if(isOvertaking){
          overtakeTimeElapsed = 0;
          overTakeDistTravelled = 0;
          initialSimTime = this->world->GetSimTime().Float();
        }
      }
    }
  }
  else if(this->carId != 1){
    if (otherCarsStart == false){
      this->SetTargetSpeed(0);
      this->MatchTargetSpeed();
    }
    else{
      this->SetTargetSpeed(5);
      this->MatchTargetSpeed();
      this->MatchTargetDirection();
    }
  }
}

//This is the Lane Driving portion
void sdcCar::laneDriving2017(){

  if (nonCurvedTurnCounter > 1000) {
    curvedTurnCounter = 0;
    averageDegree = 0;
    nonCurvedTurnCounter = 0;
  }

  if (curvedTurnCounter > 4000) {
    curvedTurnCounter = 0;
    averageDegree = 0;
  }
  double degree = this->cameraSensorData->getMidlineAngle();
  isInStraightRoad = false;
  isInCurveRoad = false;
  if(std::abs(degree) < 30){
    isInStraightRoad = true;
  }

  else if (std::abs(degree) >= 30) {
    isInCurveRoad = true;
    nonCurvedTurnCounter = 0;
  }

  if (!isInCurveRoad) {
    nonCurvedTurnCounter++;
  }
  //When its in the curve
  if (isInCurveRoad && !isInStraightRoad){
    curvedTurnCounter++;
    averageDegree *= (curvedTurnCounter - 1);
    averageDegree += degree/60;
    averageDegree /= curvedTurnCounter;
    if(degree < 0){
      this->cameraSensorData->UpdateSteeringMagnitude(degree/60);
    }
    else{
      this->cameraSensorData->UpdateSteeringMagnitude(degree/60);
    }
    this->steeringAmount = this->cameraSensorData->GetNewSteeringMagnitude();
    previousAngle = degree;
  }
  else if (isInStraightRoad && !isInCurveRoad){
    if(this->GetSpeed() < this->targetSpeed){
      this->gas = 1.0;
      this->brake = 0.0;
    }
    else{
      this->gas = 0;
    }
    if (nonCurvedTurnCounter < 400) {
      double adjustAmount = averageDegree*0.4;
      if (adjustAmount < 0) {
        adjustAmount = fmax(adjustAmount, -MAX_ADJUST);
      } else {
        adjustAmount = fmin(adjustAmount, MAX_ADJUST);
      }
    }
    if (0.4*averageDegree) {
      this->cameraSensorData->UpdateSteeringMagnitude(adjustAmount);
    } else {
      laneCenter();
    }

  } else{

    //if its about to exit curve and get back on straight road
    //lets try implementing an extra turn "counter" here so that we continue turning for a little bit long when we exit the curve
    if(std::abs(degree) <= 20){
      isInStraightRoad = true;
      isInCurveRoad = false;
      if(this->GetSpeed() < this->targetSpeed){
        this->gas = 0.7;
        this->brake = 0.0;
      }
      double verticalDifference = this->cameraSensorData->getVerticalDifference();
      if(degree < 0){
        this->cameraSensorData->UpdateSteeringMagnitude(-2);
      }
      else{
        this->cameraSensorData->UpdateSteeringMagnitude(2);
      }
      this->steeringAmount = this->cameraSensorData->GetNewSteeringMagnitude();
    }
    else if(std::abs(degree) >= 25){
      //About to enter the curve
      if (brakeTimes >= 10) {
        isInStraightRoad = false;
        isInCurveRoad = true;
        brakeTimes = 0;
      } else {
        Brake(4,4.5);
        this->cameraSensorData->UpdateSteeringMagnitude(degree/65);
        this->steeringAmount = this->cameraSensorData->GetNewSteeringMagnitude();
        brakeTimes++;
      }
    }
  }
}

/* Handles driving while lane overtaking */
void sdcCar::overtaking2017(){
  float curSimTime = this->world->GetSimTime().Float();

  //Updates distance travelled while changing lanes
  overTakeDistTravelled = (curSimTime - initialSimTime) * (this->GetSpeed());
    if((this->GetSpeed() > this->targetSpeed - 0.1) && isOvertaking){
      double estimatedRoadWidth = this->cameraSensorData->getRoadWidth();
      if((overTakeDistTravelled <= 3*estimatedRoadWidth) && (!isSideClearOfCars)){
        this->steeringAmount = -2;
      }
      else if ((overTakeDistTravelled > 3*estimatedRoadWidth) && (overTakeDistTravelled <= 6*estimatedRoadWidth) && (!isSideClearOfCars)){
        this->steeringAmount = 2;
      }
      else if (overTakeDistTravelled > 6*estimatedRoadWidth && !isSideClearOfCars){
        this->steeringAmount = 0;
        // use side lidar to decide when we can move back to right lane
        double leftMostLateral = 1e20;
        // first pass to filter out unreasonably small side objects
        std::vector<sdcVisibleObject> filteredSideObjects;
        for(int t = 0; t < rightObjects.size(); t++){
          double leftLateralDist = rightObjects[t].GetLeft().GetLateralDist();
          double rightLateralDist = rightObjects[t].GetRight().GetLateralDist();
          double lateralWidth = std::abs(leftLateralDist - rightLateralDist);
          //checks whether width of object is greater than 0.2
          if (!((lateralWidth == 0.000) && (leftLateralDist >= 0.36) && (leftLateralDist <= 0.38))){
            filteredSideObjects.push_back(rightObjects[t]);
          }
        }
        for (int t = 0; t < filteredSideObjects.size(); t++) {
          double leftLateral = filteredSideObjects[t].GetLeft().GetLateralDist();
          if(leftLateral <= leftMostLateral) {
              leftMostLateral = leftLateral;
          }
        }
        double verticalDifference = this->cameraSensorData->getVerticalDifference();
        if(leftMostLateral >= MIN_DIST_TO_PASS && std::abs(verticalDifference) < 20){
          isSideClearOfCars = true;
          overTakeDistTravelled = 0;
          initialSimTime = this->world->GetSimTime().Float();
        } else {
          laneCenter();

        }
      }
      else if (isSideClearOfCars){
        if(overTakeDistTravelled <= 3*estimatedRoadWidth){
          this->steeringAmount = 2;
        }
        else if ((overTakeDistTravelled > 3*estimatedRoadWidth) && (overTakeDistTravelled <= 6*estimatedRoadWidth) && (isSideClearOfCars)){
          this->steeringAmount = -2;
        }
        else if ((overTakeDistTravelled > 6*estimatedRoadWidth) && (overTakeDistTravelled <= 9*estimatedRoadWidth) && (isSideClearOfCars)){
          this->steeringAmount = 0;
        }
        else{
          isOvertaking = false;
          isSideClearOfCars = false;
        }
      }
  }
}

/* Assesses whether the car should attempt to overtake */
bool sdcCar::shouldWeOvertake(){
  if(this->carId == 1){
    this->SetTargetSpeed(7);
    std::vector<sdcVisibleObject> listOfFrontObjects = this->frontObjects;
    std::vector<sdcVisibleObject> filteredListOfFrontObjects;
    std::vector<sdcVisibleObject> sameLaneObjects;
    std::vector<sdcVisibleObject> otherLaneObjects;
    std::vector<sdcVisibleObject> leftLaneObjects;
    std::vector<sdcVisibleObject> rightLaneObjects;
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
    if(leftLaneObjects.size() > 0) {
      leftLaneFree = false;
    }
    if(rightLaneObjects.size() > 0) {
      carInFront = true;
    }
    if(!leftLaneFree && !isOvertaking) {
    }

    if (carInFront) {

      double closestlongitude = 100;
      for(int q = 0; q < rightLaneObjects.size(); q++){
        if (rightLaneObjects[q].GetLeft().GetLongitudinalDist() < closestlongitude){
          closestlongitude = rightLaneObjects[q].GetLeft().GetLongitudinalDist();
        }
      }
      double verticalDifference = this->cameraSensorData->getVerticalDifference();
      if (closestlongitude < MIN_DIST_TO_PASS) {
        if (std::abs(verticalDifference)< 20 && leftLaneFree) {
          turnCounter = 0;
          isOvertaking = true;
        } else {
          //don't crash if we can't pass
          this->obstacleInFront = true;
        }
      }
      else {
        this->obstacleInFront = false;
      }
    }
    return isOvertaking;
  }
  return isOvertaking;
}

/* Steers the car towards the center of the lane */
void sdcCar::laneCenter(){
  if (turnCounter > 0) {
    turnCounter++;
  }
  if (turnCounter > 3000) {
    turnCounter = 0;
  }
  double verticalDifference = this->cameraSensorData->getVerticalDifference();
  double latestAdjustAmount;

  if (std::abs(verticalDifference) > 3 && turnCounter <= 1200) {
    //shrink absolute value of vertical by 15
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
    latestAdjustAmount = -squared/4000;
    if (latestAdjustAmount > MAX_ADJUST) {
      latestAdjustAmount = fmin(latestAdjustAmount, MAX_ADJUST);
    } else {
      latestAdjustAmount = fmax(latestAdjustAmount, -MAX_ADJUST);
    }

    //set adjustAmount, which is the average angle change at beginning
    //of counter cycle
    //limit absolute value to 0.1
    if (turnCounter == 0) {
      adjustAmount = -squared/4000;
      if (adjustAmount > MAX_ADJUST) {
        adjustAmount = fmin(adjustAmount, MAX_ADJUST);
      } else {
        adjustAmount = fmax(adjustAmount, -MAX_ADJUST);
      }
      turnCounter++;
    } else {
      adjustAmount *= turnCounter;
      adjustAmount += latestAdjustAmount;
      adjustAmount /= (turnCounter + 1);
    }
    this->cameraSensorData->UpdateSteeringMagnitude(latestAdjustAmount);
    this->steeringAmount = this->cameraSensorData->GetNewSteeringMagnitude();
  } else if (turnCounter > 1200 && turnCounter < 1900) {
    if (adjustAmount > MAX_ADJUST) {
      adjustAmount = fmin(adjustAmount, MAX_ADJUST);
    } else if (adjustAmount < -1*MAX_ADJUST){
      adjustAmount = fmax(adjustAmount, -1*MAX_ADJUST);
    }

    this->cameraSensorData->UpdateSteeringMagnitude(-1*adjustAmount);
    this->steeringAmount = this->cameraSensorData->GetNewSteeringMagnitude();
  } else {
    this->cameraSensorData->UpdateSteeringMagnitude(0);
    this->steeringAmount = this->cameraSensorData->GetNewSteeringMagnitude();
    turnCounter = 0;
  }

}


/////////////////////////
////STOP SIGN STUFFS/////
/////////////////////////
void sdcCar::StopSignGridTurning(int turn){
    int progress = this->waypointProgress;
    //turn == 3 means stop
    if(turn == 3){
        this->waypointProgress++;

        this->currentState = stop;
        return;
    }
    //If the car has a reservation then proceed. If not stop and wait for a reservation

    //turn == 0 means go straight
    else if (turn == 0){
        if(STOP_SIGN_WAYPOINT_VEC[progress].hasReservation){
            this->waypointProgress++;
            this->turning = false;
            return;

        }
        else{
            if(GetSpeed() < .1){
                if(manager::stopSignHandleRequest(carId, STOP_SIGN_WAYPOINT_VEC[progress].waypointType, this->destDirection, this->fromDir)){
                    STOP_SIGN_WAYPOINT_VEC[progress].hasReservation = true;
                    this->inIntersection = true;
                    fflush(stdout);
                }
            }
            if(!laneStopped){
                manager::laneStopRequest(this->fromDir);
                laneStopped = true;
            }

            this->Stop();

            //car sends reservation request when velocity is below threshold
            //manager sends back response and car sets hasReservation to true
            //car needs to be able to send "out of intersection" message to manager
            return;
        }
    }
    //turn == 1 or 2 means turn
    else {
        if(STOP_SIGN_WAYPOINT_VEC[progress].hasReservation){
            math::Vector2d nextTarget = {STOP_SIGN_WAYPOINT_VEC[progress+1].pos.first,STOP_SIGN_WAYPOINT_VEC[progress+1].pos.second};
            sdcAngle targetAngle = AngleToTarget(nextTarget);
            this->SetTargetDirection(targetAngle);
            sdcAngle margin = this->GetOrientation().FindMargin(targetAngle);
            if(margin < .1 && margin > -.1){
                this->turning = false;
                this->waypointProgress++;
            }
        }
        else{
            if(GetSpeed() < .1){
                if(manager::stopSignHandleRequest(carId, STOP_SIGN_WAYPOINT_VEC[progress].waypointType, this->destDirection, this->fromDir)){
                    STOP_SIGN_WAYPOINT_VEC[progress].hasReservation = true;
                    this->inIntersection = true;
                    //printf("destDirection: %i\n", this->destDirection);
                    fflush(stdout);
                }
            }
            if(!laneStopped){
                manager::laneStopRequest(this->fromDir);
                laneStopped = true;
            }
            this->Stop();
            return;
        }
    }
}

void sdcCar::StopSignInitializeGraph() {
    //make the sdcIntersections
    sdcIntersection destIntersection;
    destIntersection.place = 0;
    sdcIntersection centerIntersection;
    centerIntersection.place = 1;
    turnType = genRand(2); //returns if the car goes straight (0) left (1) or right (2)
    fflush(stdout);

    if(this->x > 46 && this->x < 50){ //NORTH END

        fflush(stdout);
        //right turns first dest is 3 past intersection
        //left turns first dest is 7 past intersection
        manager::stopSignQueue(carId, 0);
        this->fromDir = 0;
        switch(turnType){
            case 0:
                destIntersection.waypoint = sdcWaypoint(0,ends[2]);
                break;
            case 1:
                destIntersection.waypoint = sdcWaypoint(0,ends[1]);
                break;
            case 2:
                destIntersection.waypoint = sdcWaypoint(0,ends[3]);
                break;
        }
        centerIntersection.waypoint = sdcWaypoint(0,std::pair<double,double>(48,55));

    }
    else if(this->y > 50 && this->y < 54){ //EAST
        manager::stopSignQueue(carId, 1);
        this->fromDir = 1;
        switch(turnType){

            case 0:
                destIntersection.waypoint = sdcWaypoint(0,ends[3]);
                break;
            case 1:
                destIntersection.waypoint = sdcWaypoint(0,ends[2]);
                break;
            case 2:
                destIntersection.waypoint = sdcWaypoint(0,ends[0]);
                break;
        }
        centerIntersection.waypoint = sdcWaypoint(0,std::pair<double,double>(55,52.5));
    }

    else if(this->x > 50 && this->x < 54){ //SOUTH
        manager::stopSignQueue(carId, 2);
        this->fromDir = 2;
        switch(turnType){
            case 0:
                destIntersection.waypoint = sdcWaypoint(0,ends[0]);
                break;
            case 1:
                destIntersection.waypoint = sdcWaypoint(0,ends[3]);
                break;
            case 2:
                destIntersection.waypoint = sdcWaypoint(0,ends[1]);
                break;
        }
        centerIntersection.waypoint = sdcWaypoint(0,std::pair<double,double>(52.5,45));
    }else if(this->y > 46 && this->y < 50){ //WEST
        manager::stopSignQueue(carId, 3);
        this->fromDir = 3;
        switch(turnType){
            case 0:
                destIntersection.waypoint = sdcWaypoint(0,ends[1]);
                break;
            case 1:
                destIntersection.waypoint = sdcWaypoint(0,ends[0]);
                break;
            case 2:
                destIntersection.waypoint = sdcWaypoint(0,ends[2]);
                break;
        }
        centerIntersection.waypoint = sdcWaypoint(0,std::pair<double,double>(45,48));
    }
    else{
        printf("woops");
        fflush(stdout);
    }


    centerIntersection.waypoint.waypointType = turnType;
    destIntersection.waypoint.waypointType = 3;
    centerIntersection.waypoint.hasReservation = false;
    //make the distance to all intersections infinity
    intersections = {destIntersection, centerIntersection};
    for (int i = 0; i < intersections.size(); ++i) {
        intersections[i].dist = std::numeric_limits<double>::infinity();
        intersections[i].place = i;
    }
}

/*
 * Drive from point to point in the given list
 */
void sdcCar::StopSignWaypointDriving() {
    int progress = this->waypointProgress;
    //printf("waypointvec size: %lu \n", STOP_SIGN_WAYPOINT_VEC.size());
    if(progress < STOP_SIGN_WAYPOINT_VEC.size()){
        // Pull the next waypoint and set the car to drive towards it

        //printf("waypointvec.size: %i", WAYPOINT_VEC.size());
        this->Accelerate();

        // Check if the car is close enough to the target to move on
        double distance = sqrt(pow(STOP_SIGN_WAYPOINT_VEC[progress].pos.first - this->x,2) + pow(STOP_SIGN_WAYPOINT_VEC[progress].pos.second - this->y,2));
        //printf("distance %f", distance);

        // CODE FROM LAST GROUP THAT ASSUMES THAT THE CAR WILL TURN ONCE WE HAVE REACHED AN INTERSECTION
        if (distance < (this->GetSpeed() * this->GetSpeed())/2.9) {
            //printf("speed: %f \n",this->GetSpeed());
            //fflush(stdout);
            this->turning = true;
        }
        if(this->turning == true){


            //USE SPEED TO DETERMINE TURNING LIM
            if (STOP_SIGN_WAYPOINT_VEC[progress].waypointType == 1) {
                //LEFT
                this->SetTurningLimit(this->GetSpeed()*6);
            } else if(STOP_SIGN_WAYPOINT_VEC[progress].waypointType == 2) {
                //RIGHT
                this->SetTurningLimit(this->GetSpeed()*19);
            }
            StopSignGridTurning(STOP_SIGN_WAYPOINT_VEC[progress].waypointType);
        } else {
            math::Vector2d nextTarget = {STOP_SIGN_WAYPOINT_VEC[progress].pos.first,STOP_SIGN_WAYPOINT_VEC[progress].pos.second};
            //printf("nextTarget.x: %f \n", WAYPOINT_VEC[1].pos.first);
            fflush(stdout);
            //printf("first: %f second: %f", WAYPOINT_VEC[progress].pos.first, WAYPOINT_VEC[progress].pos.second);
            sdcAngle targetAngle = AngleToTarget(nextTarget);
            this->SetTargetDirection(targetAngle);
            // this->LanedDriving();
        }
    } else {

        this->currentState = stop;
    }
}


//////////////////////////////////////////////////////////////
// GAZEBO METHODS - GAZEBO CALLS THESE AT APPROPRIATE TIMES //
//////////////////////////////////////////////////////////////
void sdcCar::OnUpdate() {
    if(carId == 1){
          common::Time curSimTime = this->world->GetSimTime();
          common::Time curRealTime = this->world->GetRealTime();
          float curSimTm = curSimTime.Float();
          float curRealTm = curRealTime.Float();
          float runRate = curSimTm / curRealTm;
          if((runRate < .95) && (curRealTm > .5)){
              sdcManager::setRate(runRate);
              sdcManager::setTime(curSimTm);
          }
        if(this->x > 97){
            if(this->crudeSwitch == 0){
                this->crudeSwitch = 1;
                this->SetTargetSpeed(5);
                this->MatchTargetSpeed();
                this->teleport = true;
            }
        }
    }

    //in lanedriving use 1
    //in intersection world use 2
    if(this->currentState != stop){
        if (this->crudeSwitch == 0) {
            if((this->x >= 0 && this->x <= 100) && (this->y >= 0 && this->y <= 100)){
                this->currentState = waypoint;
            }else{
                this->currentState = laneDriving;
            }
        } else if (this->crudeSwitch == 1){
            this->currentState = laneDriving;
        } else if (this->crudeSwitch == 2 || this->crudeSwitch == 3) {
            this->currentState = waypoint;
        }
    }
    if (this->crudeSwitch == 0 && !(this->x >= 0 && this->x <= 100) && (this->y >= 0 && this->y <= 100)) {
        this->currentState = laneDriving;
    }

    //Stop sign code
    if(this->crudeSwitch == 3){
        if(manager::shouldStop(carId, fromDir)){
            if(!laneStopped){
                this->currentState = stop;
                this->toldToStop = true;
            }
        }
        else{
            if(this->toldToStop){
                this->currentState = waypoint;
                this->toldToStop = false;
            }
        }


        if(this->inIntersection){
            //if outside intersection
            switch (this->destDirection) {
                  //0 = north, 1 = east, 2 = south, 3 = west
                case 0:
                    if(this->y > 55){
                        manager::stopSignCarLeft(this->carId);
                        this->inIntersection = false;
                    }
                    break;
                case 1:
                    if(this->x  > 55){
                        manager::stopSignCarLeft(this->carId);
                        this->inIntersection = false;
                    }
                    break;
                case 2:
                    if(this->y < 45){
                        manager::stopSignCarLeft(this->carId);
                        this->inIntersection = false;
                    }
                    break;
                case 3:
                    if(this->x < 45 ){
                        manager::stopSignCarLeft(this->carId);
                        this->inIntersection = false;
                    }
                    break;
            }
        }
    }

    this->velocity = this->chassis->GetWorldLinearVel();
    // Get the cars current position
    math::Pose pose = this->chassis->GetWorldPose();
    this->yaw = sdcAngle(pose.rot.GetYaw());
    this->x = pose.pos.x;
    this->y = pose.pos.y;

    if(this->frontLidarLastUpdate != this->lidarSensorData->GetLidarLastUpdate(FRONT)){
      std::vector<sdcVisibleObject> v = this->lidarSensorData->GetObjectsInFront();
      fflush(stdout);
      this->UpdateFrontObjects(v);
      this->frontLidarLastUpdate = this->lidarSensorData->GetLidarLastUpdate(FRONT);
    }

     std::vector<sdcVisibleObject> v = this->lidarSensorData->GetObjectsInFront();
     rightObjects = this->rightLidarSensorData->GetObjectsOnRight();


    // Call our Drive function, which is the brain for the car

    this->Drive();


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
    //Checks to see if the car has moved out of its starting position so we can reset another car to that location
    if (this->hasReset == 0){
        switch (this->clearIndex) {
            case 0: //North
                if (this->y < 75){
                    this->resetClear[this->clearIndex] = 1;
                    this->clearIndex = -1;
                }
            break;

            case 1: //East
                if (this->x < 75){
                    this->resetClear[this->clearIndex] = 1;
                    this->clearIndex = -1;
                }
            break;

            case 2: //South
                if (this->y > 25){
                    this->resetClear[this->clearIndex] = 1;
                    this->clearIndex = -1;
                }
            break;

            case 3: //West
                if (this->x > 25){
                    this->resetClear[this->clearIndex] = 1;
                    this->clearIndex = -1;
                }
            break;
        }
    }
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
    this->moduleSwitch = this->world->GetModel("car_wheel")->GetWorldPose().pos.x;
    this->combined = false;
    if((this->moduleSwitch < -5) && (this->moduleSwitch > -15)){
        this->crudeSwitch = 0; //For merged world
        this->combined = true;
    }
    else if((this->moduleSwitch < -15) && (this->moduleSwitch > -25)){
        this->crudeSwitch = 1; //For Lane Driving
    }
    else if((this->moduleSwitch < -25) && (this->moduleSwitch > -35)){
        this->crudeSwitch = 2; //For reservations
    }
    else if((this->moduleSwitch < -35) && (this->moduleSwitch > -45)){
        this->crudeSwitch = 3; //For stop sign
    }
    else{
        this->crudeSwitch = 1;
    }

    if(carId == 1){
        this->simStartTime = this->world->GetStartTime();
        this->setRate = false;
    }
    this->chassis = this->model->GetLink(_sdf->Get<std::string>("chassis"));
    this->camera = this->model->GetLink(_sdf->Get<std::string>("camera"));
    this->frontLidar = this->model->GetLink(_sdf->Get<std::string>("frontLidar"));
    this->frontLidarId = this->frontLidar->GetId() + 8;

    this->cameraId = this->camera->GetId() + 2;
    this->leftSideLidar = this->model->GetLink(_sdf->Get<std::string>("leftSideLidar"));
    this->leftLidarId = this->leftSideLidar->GetId() + 8;

    this->rightSideLidar = this->model->GetLink(_sdf->Get<std::string>("rightSideLidar"));
    this->rightLidarId = this->rightSideLidar->GetId() + 8;

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

    // Tell Gazebo to call OnUpdate whenever the car needs an update
    this->connections.push_back(event::Events::ConnectWorldUpdateBegin(boost::bind(&sdcCar::OnUpdate, this)));
}

/*
 * Called when the car and world are being (re)initialized.
 */
void sdcCar::Init()
{
    // Compute the angle ratio between the steering wheel and the tires
    this->steeringRatio = STEERING_RANGE / this->tireAngleRange;
    this->laneStopped = false;

    this->cameraSensorData = manager::getSensorData(this->cameraId);
    this->lidarSensorData = manager::getSensorData(this->frontLidarId);
    this->leftLidarSensorData = manager::getSensorData(this->leftLidarId);
    this->rightLidarSensorData = manager::getSensorData(this->rightLidarId);
    // During init, sensors aren't available so pull position and rotation information
    // straight from the car
    math::Pose pose = this->chassis->GetWorldPose();
    this->yaw = sdcAngle(pose.rot.GetYaw());
    this->x = pose.pos.x;
    this->y = pose.pos.y;

    this->toldToStop = false;
    this->turning = false;
    if(this->hasReset == true){
        this->numCarPass ++;
        common::Time curSimTime = this->world->GetSimTime();
        float curSimTm = curSimTime.Float();
        this->carsPerMinute = 60 * this->numCarPass / curSimTm;
        //printf("Cars Per Minute: %f\n", this->carsPerMinute);
        if (this->turnType == 0) {
            //printf("Cars Time Spent: %f, going straight\n", curSimTm - this->carStartTime);
            this->totalStraightTimeAmount += (curSimTm - this->carStartTime);
            this->carAmountStraight ++;
        } else if (this->turnType == 1){
            //printf("Cars Time Spent: %f, turning left\n", curSimTm - this->carStartTime);
            this->totalLeftTimeAmount += (curSimTm - this->carStartTime);
            this->carAmountLeft ++;
        } else if (this->turnType == 2){
            //printf("Cars Time Spent: %f, turning right\n", curSimTm - this->carStartTime);
            this->totalRightTimeAmount += (curSimTm - this->carStartTime);
            this->carAmountRight ++;
        } else {
            //printf("error: wrong dir\n");
        }
        if (curSimTm / 120 < 1.3 && curSimTm / 120 > .7 && !this->twoMinCheck) {
            //printf("---------- 2-MIN-CHECK!!! ----------\n");
            //printf("Cars Per Minute: %f\n", this->carsPerMinute);
            //printf("Average straight: %f \n", this->totalStraightTimeAmount / this->carAmountStraight);
            //printf("Average left: %f \n", this->totalLeftTimeAmount / this->carAmountLeft);
            //printf("Average right: %f \n", this->totalRightTimeAmount / this->carAmountRight);
            //printf("------------------------------------\n");
            this->twoMinCheck = true;
        }
        this->hasReset = false;
        this->currentState = waypoint;
        WAYPOINT_VEC.clear();
        STOP_SIGN_WAYPOINT_VEC.clear();
        this->waypointProgress = 0;
        this->targetSpeed = this->maxCarSpeed;
    }
    this->carStartTime = this->world->GetSimTime().Float();
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
    this->teleport = false;
    this->clearIndex = -1;
    this->hasReset = false;
    this->carIdCount ++;
    this->carId = this->carIdCount;
    this->inIntersection = false;
    this->destDirection = -1;

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
    this->obstacleInFront = false;
}
