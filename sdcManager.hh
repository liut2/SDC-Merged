#ifndef SDCINTERSECTION_HH_
#define SDCINTERSECTION_HH_

#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <exception>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/util/system.hh"
#include "manager.hh"

namespace gazebo {

    class GAZEBO_VISIBLE sdcManager : public ModelPlugin {
        // Constructor for sdcCar
        public: sdcManager();

        // These methods are called by Gazebo during the loading and initializing
        // stages of world building and populating
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void Init();

    };
}
/**
enum Direction {
  north,
  south,
  east,
  west
};

class sdcIntersection {
  public:
  // neighbor = pair.first
  // dist = pair.second
};
**/
#endif
