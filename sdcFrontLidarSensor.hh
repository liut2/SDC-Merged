#ifndef _sdcFrontLidarSensor_hh
#define _sdcFrontLidarSensor_hh

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>
#include "manager.hh"
#include "sdcSensorData.hh"

namespace gazebo
{
    class GAZEBO_VISIBLE sdcFrontLidarSensor : public SensorPlugin
    {
        public:
            virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/);
            void OnUpdate();

        private:
            sensors::RaySensorPtr parentSensor;
            event::ConnectionPtr updateConnection;
            sensors::SensorPtr sensor;
            sdcSensorData *sensorData;
            int sensorId;
    };
}

#endif
