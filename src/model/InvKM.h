#ifndef INVKM_H
#define INVKM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FwdKM.h"

typedef struct _threeDOFsInv {
	float l1, l2, l3;	// lengths of the three links
	float baseHeight;

	// /* store the coordinates of two fixed-joints */
	// float baseCoord[CART_COORD_DIM];
	// float shoulderCoord[CART_COORD_DIM];

	/* four angles between links initially, where a2 is fixed */
	float initA1, initA2, initA3, initA4;
} threeDOFsInv;


int initInvKM();

int getAllPoseByJnts(const float jntArray[JNT_NUMBER],
						float allPoss[JNT_NUMBER][POSE_FRAME_DIM]);

int getJntsByMagnetPos(const float eePos[POSE_FRAME_DIM],
						float jntArray[JNT_NUMBER]);

int finishInvKM();

#ifdef __cplusplus
}
#endif

#endif
