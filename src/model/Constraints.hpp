#include "KinematicsModel.hpp"
#ifndef MODEL_CONSTRAINTS_HPP
#define MODEL_CONSTRAINTS_HPP

#include "../C_api.hpp"
#include <math.h>

#define JA0_L -M_PI/2
#define JA0_U M_PI/2
#define JA1_L 0.0
#define JA1_U 120.0/180.0*M_PI
#define JA2_L -M_PI/2
#define JA2_U 0.0
#define GROUND_INN_RADIUS 13.5
#define GROUND_OUT_RADIUS 27.0


bool areInValidRanges(double jntAngles[NUM_OF_JOINTS],
						double result[NUM_OF_JOINTS]);

bool regulateJntAngles(double jntAngles[NUM_OF_JOINTS],
						const double delta[ACTION_DIM],
						double result[FULL_STATE_NUM_COLS]);

bool inGroundLvlWorkspace(double pos[CART_DIM]);


#endif