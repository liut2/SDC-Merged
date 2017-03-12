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

#ifndef _sdcCar_hh_
#define _sdcCar_hh_

#include <string>
#include <vector>
#include <exception>
#include <stdlib.h>
#include <time.h>
#include <random>
#include "gazebo/common/Plugin.hh"
#include <gazebo/common/common.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/util/system.hh"
#include "sdcSensorData.hh"
#include "sdcAngle.hh"
#include "sdcWaypoint.hh"
#include "sdcIntersection.hh"
#include <chrono>
#include "sdcFrontLidarSensor.hh"
#include "manager.hh"
#include "sdcManager.hh"
#include "instruction.hh"


namespace gazebo {

    class GAZEBO_VISIBLE sdcCar : public ModelPlugin {
        // Constructor for sdcCar
        public: sdcCar();
                void driveOnStraightRoad(double degree);
                void driveOnCurvedRoad(double degree);
                void laneDriving2017();
                void overtaking2017();
        // These methods are called by Gazebo during the loading and initializing
        // stages of world building and populating
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void Init();

        // Bound to Gazebo's world update, gets called every tick of the simulation
        private: void OnUpdate();

        int crudeSwitch; //Variable used to determine the modules we use
        bool combined; //Boolean to determine if we are in the combined (intersection + lanedriving) worlds
        float moduleSwitch; //Gets the position of a model in the world that sets crudeSwitch

        //Used to get performance info on intersection management
        static int numCarPass;
        static float carsPerMinute;
        static int carAmountRight;
        static int carAmountLeft;
        static int carAmountStraight;
        static float totalRightTimeAmount;
        static float totalLeftTimeAmount;
        static float totalStraightTimeAmount;
        float carStartTime;
        static bool twoMinCheck;

        // Holds the bound connection to Gazebo's update, necessary in order to properly
        // receive updates
        std::vector<event::ConnectionPtr> connections;
        std::chrono::milliseconds msStartTime;
        // The Gazebo model representation of the car
        physics::ModelPtr model;
        // Contains attribute of the world we are running
        physics::WorldPtr world;

        // Contains the wheel joints that get operated on each tick for movement
        std::vector<physics::JointPtr> joints;
        //Links to various parts of the car
        physics::LinkPtr chassis; //Used to get the car's pose and apply force
        physics::LinkPtr camera; //gets camera data
        //Lidar sensors used to detect objects
        physics::LinkPtr frontLidar;
        physics::LinkPtr leftSideLidar;
        physics::LinkPtr rightSideLidar;
        int cameraId;
        int frontLidarId;
        int leftLidarId;
        int rightLidarId;
        //sensorData object
        sdcSensorData *cameraSensorData;
        sdcSensorData *lidarSensorData;
        sdcSensorData *rightLidarSensorData;
        sdcSensorData *leftLidarSensorData;
        //Used to determine the speed the simulation is running
        common::Time  simStartTime;
        bool setRate;
        // The velocity of the car
        math::Vector3 velocity;

        //WAYPOINTS for intersection driving
        std::vector<sdcWaypoint> WAYPOINT_VEC; //List of waypoints for intersection cars
        std::vector<sdcWaypoint> STOP_SIGN_WAYPOINT_VEC;
        std::vector<sdcIntersection> intersections;

        // These variables are mostly set in the SDF for the car and relate to the
        // physical parameters of the vehicle
        double frontPower, rearPower;
        double maxSpeed;
        double maxTurnLeft;
        double maxTurnRight;
        double wheelRadius;

        double steeringRatio;
        double tireAngleRange;

        double aeroLoad;
        double swayForce;

        // The different states the car can be in. The logic and behavior of
        // the car will change depending on which state it's in, with various
        // sensor readings affecting the decision to transition states
        enum CarState { stop, waypoint, laneDriving};

        enum Direction { north, south, east, west };

        enum RelativeDirection { forward, aligned, backward, right, left };

        ///////////////////////////
        // SDC-defined variables //
        ///////////////////////////

        // The current state of the car
        CarState DEFAULT_STATE;
        CarState currentState;
        Direction currentDir;


        //For reseting the car
        math::Pose resetPose;
        bool hasReset;
        int clearIndex;
        static std::vector<int> resetClear;


        double gas; //variable that accelerates the car
        double brake; //variable that brakes the car

        // Scalars for accelrating and braking
        double accelRate;
        double brakeRate;

        // Position/rotation variables
        sdcAngle yaw;

        // Waypoint variables
        int waypointProgress;

        // Intersection variables
        bool inIntersection;
        int destDirection;
        bool laneStopped;
        int carId;
        int fromDir;
        bool toldToStop;
        static int carIdCount;
        static bool teleport;

        //Direction the car will turn once reaching the intersection
        int turnType;

        // Car limit variables
        int maxCarSpeed;
        double maxCarReverseSpeed;
        double turningLimit;

        // Flags for the car's actions
        bool turning;
        bool reversing;
        bool stopping;
        bool getSensor;
        // Movement parameters
        sdcAngle targetDirection;
        double targetSteeringAmount;
        double steeringAmount;
        double targetSpeed;


        // Follow variables
        bool isTrackingObject;
        int stationaryCount;

        // Avodiance variable
        bool obstacleInFront;

        // Variables relating to tracking objects in front of the car
        std::vector<sdcVisibleObject> frontObjects;
        int frontLidarLastUpdate;

        // The x and y position of the car
        double x;
        double y;





        /////////////////////////
        // SDC-defined methods //
        /////////////////////////

         // The 'Brain' Methods
        void Drive();
        void MatchTargetDirection();
        void MatchTargetSpeed();
        void DetectIntersection();


        void GenerateWaypoints();
        void initializeGraph();
        void StopSignInitializeGraph();
        void insertWaypointTypes(Direction startDir);


        // Driving algorithms
        void LanedDriving();
        void GridTurning(int turn);
        void WaypointDriving();
        void StopSignWaypointDriving();
        void StopSignGridTurning(int turn);

        // Helper methods
        int genRand(int max);

        void FrontLidarUpdate();
        void UpdateFrontObjects(std::vector<sdcVisibleObject> newObjects);

        sdcAngle AngleToTarget(math::Vector2d target);
        bool ObjectDirectlyAhead();
        bool IsObjectDirectlyAhead(sdcVisibleObject obj);
        float ObjectOnCollisionCourse();
        bool IsObjectOnCollisionCourse(sdcVisibleObject obj);
        bool IsObjectTooFast(sdcVisibleObject obj);
        bool IsObjectTooFurious(sdcVisibleObject obj);
        float IsGoingToHit(sdcVisibleObject obj);
        bool IsMovingForwards();
        double GetSpeed();
        sdcAngle GetDirection();
        sdcAngle GetOrientation();
        void GetNSEW();

        // Control methods
        void Accelerate(double amt = 1, double rate = 1.0);
        void Brake(double amt = 1, double rate = 1.0);
        void Stop();
        void Reverse();
        void StopReverse();

        void SetTargetDirection(sdcAngle direction);
        void SetTargetSteeringAmount(double a);
        void SetTargetSpeed(double s);
        void SetTurningLimit(double limit);

        void SetAccelRate(double rate = 1.0);
        void SetBrakeRate(double rate = 1.0);

        void combinedDriving2017();
        void overtaking2017_new();
        bool shouldWeOvertake();
        void laneCenter();
    };
}
#endif
