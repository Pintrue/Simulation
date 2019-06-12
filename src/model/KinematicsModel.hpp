#ifndef MODEL_KINEMATICSMODEL_HPP
#define MODEL_KINEMATICSMODEL_HPP

#include "FwdKM.h"

#define NUM_OF_JOINTS 3


class KinematicsModel {
    public:
        KinematicsModel();  // constructor
        
        bool getPoseByJnts(const float jnts[NUM_OF_JOINTS], float pose[6]);
		bool getAllPoss(const float jnts[NUM_OF_JOINTS], float allPoss[POSS_NUMBER][POSE_FRAME_DIM]);
		bool getJntsByPose(const float pose[POSE_FRAME_DIM], float jnts[NUM_OF_JOINTS]);

        float _jointAngles[NUM_OF_JOINTS]; // angles between segments
        float _cartPose[POSE_FRAME_DIM];       // current end-effector position
};


#endif
