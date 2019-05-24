#include <stdio.h>
#include "KinematicsModel.hpp"
#include "../utils/Utils.hpp"
#include <iostream>


using namespace std;


KinematicsModel::KinematicsModel() {}

/*
*	Given the joint angles the arm needs to move to,
*	calculate the final position of the end-effector
*	(also known as the tip of delivery part) in Car-
*	tesian coordinate form.
*/
bool KinematicsModel::getPoseByJnts(const double jnts[NUM_OF_JOINTS],
										double pose[POSE_FRAME_DIM]) {
	initFwdKM();
	if (getEEPoseByJnts(jnts, pose) >= 0) {
		for (int i = 0; i < NUM_OF_JOINTS; ++i) {
			_jointAngles[i] = jnts[i];
		}
		for (int i = 0; i < POSE_FRAME_DIM; ++i) {
			_cartPose[i] = pose[i];
		}
		finishFwdKM();
		return true;
	} else {
		finishFwdKM();
		cout << "Angle out of bound, at function getPoseByJnts() in file "
				<< "KinematicsModel.cpp." << endl;
		return false;
	}
	
}


bool KinematicsModel::getAllPoss(const double jnts[NUM_OF_JOINTS], double allPoss[POSS_NUMBER][POSE_FRAME_DIM]) {
	initFwdKM();
	if (getAllPossByJnts(jnts, allPoss) >= 0) {
		finishFwdKM();
		return true;
	} else {
		finishFwdKM();
		cout << "Angle out of bound, at function getAllPossByJnts() in file "
				<< "KinematicsModel.cpp." << endl;
		return false;
	}
}


// bool KinematicsModel::getJntsByPose(const double pose[6],
// 										JntArray& jnts) {
// 	ChainIkSolverPos_LMA iKSolver =
// 		ChainIkSolverPos_LMA(_kdlChain);
	
// 	Frame eeFrame;
// 	for (int i = 0; i < 3; ++i) {
// 		eeFrame.p(i) = pose[i];
// 	}
// 	if (iKSolver.CartToJnt(_jointAngles, eeFrame, jnts)) {
// 		_cartPose = eeFrame;
// 		_jointAngles = jnts;
// 		return true;
// 	} else {
// 		return false;
// 	}	
// }