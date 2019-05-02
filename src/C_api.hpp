#ifndef C_API_HPP
#define C_API_HPP

#include "robot_reinforcement_learning/C/matrix_op.h"

extern "C" int initEnv();

// state: angles, target/coord, ee/coord, if succeed = 1*10 double
extern "C" matrix_t* resetState(int randAngle, int destPos);

// action::1*3 double
extern "C" matrix_t* step(matrix_t* action);

#endif