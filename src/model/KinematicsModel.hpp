#ifndef MODEL_KINEMATICSMODEL_HPP
#define MODEL_KINEMATICSMODEL_HPP

#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>

#define NUM_OF_JOINTS 3


class KinematicsModel {
    public:
        KinematicsModel();  // constructor
        
        void init(double origin[3]);
        bool fwdKmt(const KDL::JntArray& jointAngles, KDL::Frame& eeFrame);
		bool invKmt(const KDL::Frame& eeFrame, KDL::JntArray& jnts);

        KDL::Chain _kdlChain;       // data structure for fk model
        KDL::JntArray _jointAngles; // angles between segments
        KDL::Frame _cartPose;       // current end-effector position
};


#endif