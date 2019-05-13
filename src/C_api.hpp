#ifndef C_API_HPP
#define C_API_HPP

#define CART_DIM 3
#define FULL_STATE_NUM_COLS 11
#define ACTION_DIM 3
#define TIP_REACHED_RANGE 5.0

#include "robot_reinforcement_learning/C/matrix_op.h"
#include "Sim.hpp"

extern "C" int initEnv();

extern "C" double rand_uniform(double low, double high);
// state: angles, target/coord, ee/coord, if succeed = 1*10 double
extern "C" matrix_t* resetState(int randAngle, int destPos);

// three angles to move about, NOT the angles to be moved to
// action::1*3 double
extern "C" matrix_t* step(matrix_t* action);

#endif
