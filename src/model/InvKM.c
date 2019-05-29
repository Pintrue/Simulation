#include "InvKM.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

static threeDOFsInv* arm;


int initInvKM() {
	/* DESCRIPTION of the arm */
	double linkLength[3] = {sqrt(3.5*3.5+3.9*3.9), sqrt(1.7*1.7+10.5*10.5), sqrt(3.5*3.5+16.5*16.5)};
	double initJntAngles[3] = {0.0, atan2(1.7, 10.50), atan2(3.5, 16.5)};
	double baseHeight = 2.9;


	arm = (threeDOFsInv*) malloc(sizeof(threeDOFsInv));

	arm->l1 = linkLength[0];
	arm->l2 = linkLength[1];
	arm->l3 = linkLength[2];

	arm->baseHeight = baseHeight;

	arm->initA1 = initJntAngles[0];
	arm->initA2 = atan2(3.5, 3.9);
	arm->initA3 = initJntAngles[1];
	arm->initA4 = initJntAngles[2];

	return 0;
}


int getJntsByEEPos(const double eePos[POSE_FRAME_DIM], double jntArray[JNT_NUMBER]) {
	double d1 = atan2(eePos[0], eePos[2]);
	// printf("d1 is equal to %f\n", d1);

	// /* calculate the coordinate of shoulder arm */
	// double shoulderCoord[CART_COORD_DIM];
	// /* in side-view */
	// double sideX = arm->l1 * cos(arm->initA2); // 2D x-position
	// double shoulderY = arm->l1 * sin(arm->initA2) + arm->baseHeight;	// 2D y-position

	// double shoulderX = sideX * sin(d1);
	// double shoulderZ = sideX * cos(d1);

	// shoulderCoord[0] = shoulderX; shoulderCoord[1] = shoulderY; shoulderCoord[2] = shoulderZ;

	initFwdKM();
	double angle[3] = {0, 0, 0};
	double jntPoss[2][3];
	getJntPosByAngle(angle, jntPoss, 1);
	finishFwdKM();

	double shoulderSideX = sqrt(pow(jntPoss[0][0], 2) + pow(jntPoss[0][2], 2));
	double shoulderY = jntPoss[0][1];
	double eeSideX = sqrt(pow(eePos[0], 2) + pow(eePos[2], 2));

	double dShoulderEEX = eeSideX - shoulderSideX;
	double dShoulderEEY = eePos[1] + MAGNET_EE_HEIGHT_OFFSET + WRIST_ROTATE_HEIGHT_OFFSET * cos(getCache()->a4) - shoulderY;
	double dShoulderEEAngle = atan2(dShoulderEEY, dShoulderEEX);

	double l4 = sqrt(pow(dShoulderEEX , 2) + pow(dShoulderEEY, 2));
	/* use law of cosine to solve the triangle formed by shoulder, forearm joints and EE */
	double loC3 = acos((pow(arm->l2, 2) + pow(l4, 2) - pow(arm->l3, 2)) / (2 * arm->l2 * l4));
	double loC4 = acos((pow(arm->l2, 2) + pow(arm->l3, 2) - pow(l4, 2)) / (2 * arm->l2 * arm->l3));

	double d2 = M_PI - dShoulderEEAngle - loC3 - arm->initA3;
	double d3 = arm->initA3 + arm->initA4 - loC4;

	printf("Joint angles from Inverse: %f %f %f\n", d1, d2, d3);
	if (d1 > JNT0_U || d1 < JNT0_L ||
		d2 > JNT1_U || d2 < JNT1_L ||
		d3 > JNT2_U || d3 < JNT2_L) {
		return JNT_ANGLES_OUT_OF_BOUND;
	}

	jntArray[0] = d1;
	jntArray[1] = d2;
	jntArray[2] = d3;
	return 0;
}


int finishInvKM() {
	free(arm);
	return 0;
}


// int main() {
// 	initInvKM();

// 	double eePosInv[POSE_FRAME_DIM] = {3.760000, 21.640000, 6.880000};
// // 	double eePos[POSE_FRAME_DIM] = {-3.563496e+00, 5.000000e-01, 1.765392e+01};
// // 	// double eePos[POSE_FRAME_DIM] = {-2.360000, 11.800000, 13.460000};
// 	double jntArray[JNT_NUMBER];

// 	if (getJntsByEEPos(eePosInv, jntArray) >= 0) {
// 		printf("[ ");
// 		for (int i = 0; i < 3; ++i) {
// 			printf("%f ", jntArray[i]);
// 		}
// 		printf("]\n");
// 	} else {
// 		printf("EE Pos not in eligible range.\n");
// 	}

// 	initFwdKM();
// 	double angle[3] = {0.5, 0.6, -1.20};
// 	// double allPoss[2][3];
// 	// getJntPosByAngle(angle, allPoss, 2);
// 	// for (int i = 0; i < 2; ++i) {
// 	// 	printf("[ ");
// 	// 	for (int j = 0; j < 3; ++j) {
// 	// 		printf("%f ", allPoss[i][j]);
// 	// 	}
// 	// 	printf("]\n");
// 	// }
// 	double eePos[6];
// 	getEEPoseByJnts(angle, eePos);
// 	printf("[ ");
// 	for (int i = 0; i < 3; ++i) {
// 		printf("%f ", eePos[i]);
// 	}
// 	printf("]\n");
// }
