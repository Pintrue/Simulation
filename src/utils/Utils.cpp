//
// Created by Pinchu on 2019/3/7.
//
#include <stdio.h>
#include "Utils.hpp"

using namespace std;


void printFrame(const KDL::Frame& eeFrame) {// Print the frame
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++) {
			double a = eeFrame(i, j);
			if (a < 0.0001 && a > -0.001) {
				a = 0.0;
			}
			cout << setprecision(4) << a << "\t\t";
		}
		cout << endl;
	}
}


void convFrameToPose(const KDL::Frame& frame, double pose[POSE_DIM]) {
	/**
	 * Pose is consist of cartesian coordinate and
	 * orientation in the form of Euler angle, e.g.
	 * 
	 * 	pose => (x, y, z, alpha, beta, gamma)
	 * 
	**/

	for (int i = 0; i < 3; ++i) {
		pose[i] = frame.p(i);
	}

	double alp, bet, gam;	// Euler angles Z, Y, X
	frame.M.GetEulerZYX(alp, bet, gam);
	pose[3] = gam; pose[4] = bet; pose[5] = alp;
}