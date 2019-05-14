#ifndef C_API_HPP
#define C_API_HPP

#define CART_DIM 3
#define FULL_STATE_NUM_COLS 11
#define ACTION_DIM 3
#define TIP_REACHED_RANGE 5.0

#include "robot_reinforcement_learning/C/matrix_op.h"
#include "robot_reinforcement_learning/C/main.h"
#include "Sim.hpp"

extern "C" int initEnv(int act_dim);

extern "C" double rand_uniform(double low, double high);
// state: angles, target/coord, ee/coord, if succeed = 1*10 double
extern "C" matrix_t* resetState(int randAngle, int destPos, int state_dim, int act_dim);

// three angles to move about, NOT the angles to be moved to
// action::1*3 double
extern "C" matrix_t* step(matrix_t* action, int state_dim, int act_dim);

extern "C" void closeEnv(int state_dim, int act_dim);

extern "C" matrix_t* random_action(int state_dim, int act_dim);

#endif
