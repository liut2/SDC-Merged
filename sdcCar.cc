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
    std::pair<double,double>(90, 48),
    std::pair<double,double>(48, 10),
    std::pair<double,double>(10,52.5)};
//n, e, s, w
//dijkstra's stuff
std::vector<int> unvisited;

const int size = 5;
//std::pair<double,double> destination = {50,50};

//std::vector<std::pair<double,double>> destinations;


int sdcCar::carIdCount = 0;


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


//     If not in avoidance, check if we should start following the thing
//     in front of us. If following is done, kick out to default state
    if(this->currentState != intersection){
        //printf("in if\n");
        // If there's a stop sign, assume we're at an intersection
//        if(this->ignoreStopSignsCounter == 0 && sdcSensorData::stopSignFrameCount > 5){
//            this->currentState = intersection;
//        }

        // If something is ahead of us, default to trying to follow it
//        if (this->ObjectDirectlyAhead()){
//            printf("currentstate = follow\n");
//            this->currentState = follow;
//        }else if(this->currentState == follow && !this->isTrackingObject){
//            this->currentState = waypoint;;
//        }

        // Look for objects in danger of colliding with us, react appropriately
//        if (this->ObjectOnCollisionCourse()){
//            this->currentState = avoidance;
//        }
    } else {
      // road-driving
      //currentState = road-driving;
    }

    //this->ignoreStopSignsCounter = fmax(this->ignoreStopSignsCounter - 1, 0);


    // Possible states: stop, waypoint, intersection, follow, avoidance

    switch(this->currentState)
    {
        // Final state, car is finished driving

        case stop:
            //printf("in stop\n");
            this->Stop();
        break;

        // Default state; drive straight to target location
        case waypoint:

        // Handle lane driving

//          this->Accelerate();
        //  this->Stop();
           // printf("inWaypoint\n");
            this->WaypointDriving(WAYPOINT_VEC);
            
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

        // Smarter way to avoid objects; stopping, swerving, etc.
        case laneDriving:
            //printf("its driving in a lane\n");
        break;

    }
    this->MatchTargetDirection();
    // Attempts to match the target speed
    this->MatchTargetSpeed();
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
        this->steeringAmount = this->sensorData->GetNewSteeringMagnitude();
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
void sdcCar::WaypointDriving(std::vector<sdcWaypoint> WAYPOINT_VEC) {
    int progress = this->waypointProgress;
    //printf("waypointvec size: %lu \n", WAYPOINT_VEC.size());
    if(progress < WAYPOINT_VEC.size()){
        // Pull the next waypoint and set the car to drive towards it
        //printf("waypointvec.size: %i", WAYPOINT_VEC.size());
        this->Accelerate();

        // Check if the car is close enough to the target to move on
        double distance = sqrt(pow(WAYPOINT_VEC[progress].pos.first - this->x,2) + pow(WAYPOINT_VEC[progress].pos.second - this->y,2));
        //printf("distance %f", distance);

        // CODE FROM LAST GROUP THAT ASSUMES THAT THE CAR WILL TURN ONCE WE HAVE REACHED AN INTERSECTION
        if (distance < (this->GetSpeed() * this->GetSpeed())/2.9) {
            //printf("speed: %f \n",this->GetSpeed());
            //fflush(stdout);
            this->turning = true;
        }
        if(this->turning == true){


            //USE SPEED TO DETERMINE TURNING LIM
            if (WAYPOINT_VEC[progress].waypointType == 1) {
                //LEFT
                this->SetTurningLimit(this->GetSpeed()*6);
            } else if(WAYPOINT_VEC[progress].waypointType == 2) {
                //RIGHT
                this->SetTurningLimit(this->GetSpeed()*19);
            }
            GridTurning(WAYPOINT_VEC[progress].waypointType);
        } else {
            math::Vector2d nextTarget = {WAYPOINT_VEC[progress].pos.first,WAYPOINT_VEC[progress].pos.second};
            //printf("nextTarget.x: %f \n", WAYPOINT_VEC[1].pos.first);
            fflush(stdout);
            //printf("first: %f second: %f", WAYPOINT_VEC[progress].pos.first, WAYPOINT_VEC[progress].pos.second);
            sdcAngle targetAngle = AngleToTarget(nextTarget);
            this->SetTargetDirection(targetAngle);
            // this->LanedDriving();
        }
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
    int lanePos = this->sensorData->LanePosition();
    this->SetTurningLimit(this->sensorData->GetNewSteeringMagnitude());

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
    float distance = std::abs(destX - this->x) + std::abs(destY - this->y);
    
    if(turn == 3){
        this->waypointProgress++;

        this->currentState = stop;
        return;
    }
    //If the car has a reservation then proceed. If not stop and wait for a reservation

    //turn == 0 means go straight
    else if (turn == 0){
       

        if(distance < 5){
            this->waypointProgress = 1;
            this->turning = false;
            printf("turn == 0\n");
            return;

        }
        else{
            if (distance < 15){
                if(!WAYPOINT_VEC[progress].hasReservation){
                    auto instruction = sdcManager::reservationRequest(carId, this->x, this->y, GetSpeed(), WAYPOINT_VEC[progress].waypointType, this->destDirection, this->fromDir);
                    this->targetSpeed = instruction.getSpeed();
                    if (instruction.getHasReservation() == 1){
                        WAYPOINT_VEC[progress].hasReservation = true;
                        //printf("straight got reservation\n");
                    }
                }
            }
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
            if(!WAYPOINT_VEC[progress].hasReservation){
                this->Stop();
            }
            

            //car sends reservation request when velocity is below threshold
            //sdcManager sends back response and car sets hasReservation to true
            //car needs to be able to send "out of intersection" message to sdcManager
            return;
        }
    }
    //turn == 1 or 2 means turn
    else {
        if(distance < 3 && WAYPOINT_VEC[progress].hasReservation){
            math::Vector2d nextTarget = {WAYPOINT_VEC[progress+1].pos.first,WAYPOINT_VEC[progress+1].pos.second};
//            printf("next target pos.first: %f\n",WAYPOINT_VEC[progress+1].pos.first);
//            printf("next target pos.second: %f\n",WAYPOINT_VEC[progress+1].pos.second);
            fflush(stdout);
            sdcAngle targetAngle = AngleToTarget(nextTarget);
            this->SetTargetDirection(targetAngle);
            sdcAngle margin = this->GetOrientation().FindMargin(targetAngle);
            if(margin < .1 && margin > -.1){
                this->turning = false;
                this->waypointProgress++;
            }
        }
        if (distance < 15){
            if(!WAYPOINT_VEC[progress].hasReservation){
                auto instruction = sdcManager::reservationRequest(carId, this->x, this->y, GetSpeed(), WAYPOINT_VEC[progress].waypointType, this->destDirection, this->fromDir);
                this->targetSpeed = instruction.getSpeed();
                if (instruction.getHasReservation() == 1){
                    WAYPOINT_VEC[progress].hasReservation = true;
                    printf("turning got reservation");
            }
            }
        }
        if(!WAYPOINT_VEC[progress].hasReservation){
            this->Stop();
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
    WAYPOINT_VEC.push_back(intersections[1].waypoint);
    WAYPOINT_VEC.push_back(intersections[0].waypoint);
   // printf("end of genwaypoints\n");
}


//nesw
void sdcCar::initializeGraph() {
    //make the sdcIntersections
    sdcIntersection destIntersection;
    destIntersection.place = 0;
    sdcIntersection centerIntersection;
    centerIntersection.place = 1;
    int turnType = genRand(2); //returns if the car goes straight (0) left (1) or right (2)
    printf("turnType: %i carId: %i\n", turnType, this->carId);
    fflush(stdout);

    if(this->x > 46 && this->x < 50){ //NORTH END
        printf("idfirst:%i",carId);
        fflush(stdout);
        //right turns first dest is 3 past intersection
        //left turns first dest is 7 past intersection
        sdcManager::stopSignQueue(carId, 0);
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
        sdcManager::stopSignQueue(carId, 1);
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
        sdcManager::stopSignQueue(carId, 2);
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
        sdcManager::stopSignQueue(carId, 3);
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
    printf("centerWPtype: %i\n", centerIntersection.waypoint.waypointType);
    intersections = {destIntersection, centerIntersection};
    for (int i = 0; i < intersections.size(); ++i) {
        intersections[i].dist = std::numeric_limits<double>::infinity();
        intersections[i].place = i;
    }
    printf("end of graph\n");
}

void sdcCar::insertWaypointTypes(Direction startDir) {
    Direction curDir = startDir;
    int nextDir = intersections[1].waypoint.waypointType;
    printf("nextDir: %i", nextDir);
    int current = 1;
    int next = 0;
  // get the direction the car heads in from the current intersection to
  // the next one
    // 0 = straight, 1 = left, 2 = right, 3 = stop
    switch (curDir) {
      case north:
        printf("north\n");
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
            printf("east\n");
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
        printf("south");
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
        printf("west");
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
bool sdcCar::ObjectOnCollisionCourse(){
    if(this->frontObjects.size() == 0) return false;

    for (int i = 0; i < this->frontObjects.size(); i++) {
        if(this->IsObjectOnCollisionCourse(this->frontObjects[i])){
            return true;
        }
    }
    return false;
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
    //printf("--stopping bool: %i--", this->stopping);
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

//////////////////////////////////////////////////////////////
// GAZEBO METHODS - GAZEBO CALLS THESE AT APPROPRIATE TIMES //
//////////////////////////////////////////////////////////////
void sdcCar::OnUpdate()
{
    //printf("\nin onupdate\n");
    //    if(this->getSensor){
    //
    //        this->getSensor = false;
    //    }
    //this->sensorData = sdcManager::getSensorData(carId);
    //REMEMBER TO CHANGE THIS
    int crudeSwitch = 2; //in merged world use 0
    //in lanedriving use 1
    //in intersection world use 2
    
    if (crudeSwitch == 0) {
        if((this->x >= 0 && this->x <= 100) && (this->y >= 0 && this->y <= 100)){
            //printf("starting at %f,%f \n", this->x, this->y);
            this->currentState = waypoint;
        }else{
            
            this->currentState = laneDriving;
        }
    } else if (crudeSwitch == 1){
        if((this->x <= 10 && this->x >= -10) && (this->y <= 10 && this->y >= -10)){
            //printf("starting at %f,%f \n", this->x, this->y);
            this->currentState = laneDriving;
        }
        this->currentState = laneDriving;
    } else if (crudeSwitch == 2) {
        //printf("Crude switch == 2 : for intersection worlds");
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
    //    if(this->frontLidarLastUpdate != this->sensorData->GetLidarLastUpdate(FRONT)){
    //        printf("updating front objects\n");
    //        std::vector<sdcVisibleObject> v = this->sensorData->GetObjectsInFront();
    //        //printf("visibleobjects size: %lu\n", v.size());
    //        fflush(stdout);
    //        this->UpdateFrontObjects(v);
    //        this->frontLidarLastUpdate = this->sensorData->GetLidarLastUpdate(FRONT);
    //    }
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
    
    // Call our Drive function, which is the brain for the car
    if(this->carId == 1){
        this->Drive();
    }
    else{
        this->Stop();
    }
    
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
    this->chassis = this->model->GetLink(_sdf->Get<std::string>("chassis"));
    this->camera = this->model->GetLink(_sdf->Get<std::string>("camera"));
    this->cameraId = this->camera->GetId() + 2;
    //printf("camera ID: %i\n",this->cameraId);

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
    this->sensorData = manager::getSensorData(cameraId);
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

    this->maxSpeed = 10;
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
    this->targetSpeed = 6;

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
