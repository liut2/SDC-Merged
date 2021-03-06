//Class that handles registering and updating the front lidar sensor


#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>
#include "sdcFrontLidarSensor.hh"

using namespace gazebo;
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(sdcFrontLidarSensor)

// Pointer to the update event connection
event::ConnectionPtr updateConnection;
sensors::RaySensorPtr parentSensor;

////// LIDAR ON FRONT OF CAR
void sdcFrontLidarSensor::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
    //this->lidarId = this->sensor->GetId();
    this->sensor = _sensor;
    //printf("lidar sensor id: %i\n",this->sensor->GetId());
    this->sensorId = this->sensor->GetId();
    //printf("fLidar sensorId: %i\n", this->sensorId);

    //printf("this camerasId: %i \n", this->cameraId);
    this->sensorData = manager::getSensorData(sensorId);
    // Get the parent sensor.
    this->parentSensor = boost::dynamic_pointer_cast<sensors::RaySensor>(_sensor);

    // Make sure the parent sensor is valid.
    if (!this->parentSensor)
    {
        gzerr << "Couldn't find a laser\n";
        return;
    }

    // Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(boost::bind(&sdcFrontLidarSensor::OnUpdate, this));

    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);

    this->sensorData->InitLidar(FRONT, this->parentSensor->AngleMin().Radian(), this->parentSensor->GetAngleResolution(), this->parentSensor->GetRangeMax(), this->parentSensor->GetRayCount());
}

// Called by the world update start event
void sdcFrontLidarSensor::OnUpdate(){
    std::vector<double>* rays = new std::vector<double>();
    for (unsigned int i = 0; i < this->parentSensor->GetRayCount(); ++i){
        rays->push_back(this->parentSensor->GetRange(i));
    }
  //  printf("onupdate frontlidar\n");
  //  printf("first ray range: %f\n", this->parentSensor->GetRange(360));
    this->sensorData->UpdateLidar(FRONT, rays);
    //printf("The sensor id is %i and the size of rays is %i\n", this->sensor->GetId(), rays->size());
}
