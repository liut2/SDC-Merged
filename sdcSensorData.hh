#ifndef _sdcSensorData_hh
#define _sdcSensorData_hh

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>
#include <map>
#include "sdcAngle.hh"
#include "sdcLidarSensorInfo.hh"
#include "sdcLidarRay.hh"
#include "sdcVisibleObject.hh"

namespace gazebo
{
    // An enumeration of all positions of Lidar sensors, including a few (TOP, SIDE_LEFT, SIDE_RIGHT) that
    // correspond to a collection of sensors
    enum LidarPos {FRONT, BACK, TOP, TOP_FORWARD, TOP_RIGHT, TOP_BACKWARD, TOP_LEFT, SIDE_LEFT, SIDE_RIGHT, SIDE_LEFT_FRONT, SIDE_LEFT_BACK, SIDE_RIGHT_FRONT, SIDE_RIGHT_BACK};

    class sdcSensorData
    {
        // Lidar variables and methods
    public:
        sdcSensorData();
        sdcSensorData(int id);
        void InitLidar(LidarPos lidar, double minAngle, double angleResolution, double maxRange, int numRays);
        void UpdateLidar(LidarPos lidar, std::vector<double>* newRays);
        std::vector<double> GetLidarRays(LidarPos lidar);
        void UpdateCameraData(int lanePos);
        int LanePosition();
        void UpdateSteeringMagnitude(double steerMag);

        double GetNewSteeringMagnitude();

        std::vector<sdcLidarRay> GetBlockedFrontRays();
        std::vector<sdcLidarRay> GetBlockedBackRays();
        std::vector<sdcVisibleObject> GetObjectsInFront();

        int GetLidarLastUpdate(LidarPos lidar);
        int GetLidarNumRays(LidarPos lidar);
        sdcAngle GetLidarMinAngle(LidarPos lidar);
        double GetLidarAngleResolution(LidarPos lidar);
        double GetLidarMaxRange(LidarPos lidar);

        sdcAngle backMinAngle;
        double backAngleResolution;

        int frontLidarLastUpdate;

        int stopSignFrameCount;
        double sizeOfStopSign;
        bool stopSignInLeftCamera;
        bool stopSignInRightCamera;
        int lanePosition;
        double newSteerMagnitude;

        // GPS variables and methods
        double gpsX;
        double gpsY;
        int sensorId;
        sdcAngle gpsYaw;

        math::Vector2d GetPosition();
        sdcAngle GetYaw();
        
        void UpdateGPS(double x, double y, double yaw);

    private:
        std::vector<double>* frontLidarRays;
        std::vector<double>* backLidarRays;

        std::vector<double>* topLeftLidarRays;
        std::vector<double>* topRightLidarRays;
        std::vector<double>* topForwardLidarRays;
        std::vector<double>* topBackwardLidarRays;

        std::vector<double>* sideLeftFrontLidarRays;
        std::vector<double>* sideLeftBackLidarRays;
        std::vector<double>* sideRightFrontLidarRays;
        std::vector<double>* sideRightBackLidarRays;

        std::map<LidarPos, sdcLidarSensorInfo> lidarInfoMap;
        
    };
}

#endif
