#ifndef SIM_HPP
#define SIM_HPP

#include "model/KinematicsModel.hpp"
#include "model/Trajectory.hpp"

class Sim {
    public:
        KinematicsModel _km;// kinematics model
        Trajectory _tjt;    // trajectory model
};

#endif