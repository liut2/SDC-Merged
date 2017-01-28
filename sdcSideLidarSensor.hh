#ifndef _sdcSideLidarSensor_hh
#define _sdcSideLidarSensor_hh

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>
#include "sdcSensorData.hh"
#include "manager.hh"

namespace gazebo
{
    class GAZEBO_VISIBLE sdcSideLidarSensor : public SensorPlugin
    {
        public:
            virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/);
            void OnUpdate();

        private:
            sensors::RaySensorPtr parentSensor;
            event::ConnectionPtr updateConnection;
            LidarPos lidarPos;
            sensors::SensorPtr sensor;
            sdcSensorData *sensorData;
            int sensorId;
    };
}

#endif
