#ifndef MODEL_KINEMATICSMODEL_HPP
#define MODEL_KINEMATICSMODEL_HPP

#include "FwdKM.h"

#define NUM_OF_JOINTS 3


class KinematicsModel {
    public:
        KinematicsModel();  // constructor
        
        bool getPoseByJnts(const double jnts[NUM_OF_JOINTS], double pose[6]);
		bool getAllPoss(const double jnts[NUM_OF_JOINTS], double allPoss[POSS_NUMBER][POSE_FRAME_DIM]);
		bool getJntsByPose(const double pose[POSE_FRAME_DIM], double jnts[NUM_OF_JOINTS]);

        double _jointAngles[NUM_OF_JOINTS]; // angles between segments
        double _cartPose[POSE_FRAME_DIM];       // current end-effector position
};


#endif
