#include <stdio.h>
#include "Utils.hpp"
#include <iostream>
#include <iomanip>
#include <math.h>
#include "../model/FwdKM.h"

using namespace std;


// void printFrame(const KDL::Frame& eeFrame) {// Print the frame
// 	for (int i = 0; i < 4; i++){
// 		for (int j = 0; j < 4; j++) {
// 			float v = eeFrame(i, j);
// 			if (v < 0.0001 && v > -0.001) {
// 				v = 0.0;
// 			}
// 			cout << setprecision(4) << v << "\t\t";
// 		}
// 		cout << endl;
// 	}
// }


void printPose(const float eePos[POSE_DIM]) {
	for (int i = 0; i < 3; ++i) {
		float v = eePos[i];
		if (v < 0.0001 && v > -0.001) {
			v = 0.0;
		}
		cout << setprecision(4) << v << "\t\t";
	}
	cout << endl;
	for (int i = 3; i < 6; ++i) {
		float v = eePos[i];
		if (v < 0.0001 && v > -0.001) {
			v = 0.0;
		}
		cout << setprecision(4) << v << "\t\t";
	}
	cout << endl;
}


// void convFrameToPose(const KDL::Frame& frame, float pose[POSE_DIM]) {
// 	/**
// 	 * Pose is consist of cartesian coordinate and
// 	 * orientation in the form of Euler angle, e.g.
// 	 * 
// 	 * 	pose => (x, y, z, alpha, beta, gamma)
// 	 * 
// 	**/

// 	for (int i = 0; i < 3; ++i) {
// 		pose[i] = frame.p(i);
// 	}

// 	float z, y, x;	// Euler angles Z, Y, X
// 	frame.M.GetEulerZYX(z, y, x);
// 	pose[3] = x; pose[4] = y; pose[5] = z;
// }


// void printJntArray(const KDL::JntArray& ja) {
// 	cout << "The requested joint array: " << endl;
// 	cout << "[ ";
// 	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
// 		cout << setprecision(4) << ja(i) << " ";
// 	}
// 	cout << "]" << endl;
// }

bool withinCylinder(float center[CART_COORD_DIM], int radius,
					float obj[CART_COORD_DIM]) {
	float distToCenter = sqrt(pow(center[0] - obj[0], 2)
								+ pow(center[2] - obj[2], 2));
	return distToCenter <= (float) radius;
}


bool illegalJntBoundary(const float* jntArray) {
	return (jntArray[0] > JNT0_U || jntArray[0] < JNT0_L ||
			jntArray[1] > JNT1_U || jntArray[1] < JNT1_L ||
			jntArray[2] > JNT2_U || jntArray[2] < JNT2_L);
}
