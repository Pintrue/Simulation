#ifndef MODEL_KINEMATICSMODEL_HPP
#define MODEL_KINEMATICSMODEL_HPP

#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>


class KinematicsModel {
    public:
        KinematicsModel();  // constructor

        void init(double origin[3]);
        bool jntsToCart(const KDL::JntArray& jointAngles, KDL::Frame& eeFrame);

    private:
        KDL::Chain _kdlChain;   // data structure for kine. model
        KDL::JntArray _jointAngles; // angles between segments
        KDL::Frame _cartPose;   // current end-effector position
};


#endif