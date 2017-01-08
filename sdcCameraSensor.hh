#ifndef _sdcCameraSensor_hh
#define _sdcCameraSensor_hh

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>
#include <iostream>

#include "sdcSensorData.hh"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace gazebo
{
    class GAZEBO_VISIBLE sdcCameraSensor : public SensorPlugin
    {
        public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/);
        public: void OnUpdate();
        public: double getSlope(cv::Vec4i l);
        public: cv::Point getIntersectionPoint(cv::Vec4i l1, cv::Vec4i l2);
        public: double getPointLineDist(cv::Point p1, cv::Vec4i l1);
        public: bool isTooClose(cv::Vec4i leftLine, cv::Vec4i rightLine, int i, double row, double col);
        public: cv::Vec4i extendLine(cv::Vec4i line, cv::Vec4i topHorizontal, cv::Vec4i bottomHorizontal);
        public: cv::Vec4i createLine(double x1, double y1, double x2, double y2);
        public: double getNewTurningAngle(cv::Vec4i midLine);
        private: sensors::MultiCameraSensorPtr parentSensor;
        private: event::ConnectionPtr updateConnection;
    };
}

#endif
