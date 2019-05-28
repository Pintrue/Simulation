#ifndef C_API_HPP
#define C_API_HPP

#define CART_DIM 3
#define TIP_REACHED_RANGE 5.0
#define OBJ_ATTACHED_RANGE 1.0
#define OBJ_NEAR_DEST_RANGE 3.0
#define OBJ_HEIGHT 0.5

// #define REACHING
#define PICK_N_PLACE // testing now, DELETE LATER!!!!

#ifdef REACHING
	#define FULL_STATE_NUM_COLS 14
	#define ACTION_DIM 3
#endif

#ifndef REACHING
	#ifdef PICK_N_PLACE
		#define FULL_STATE_NUM_COLS 19
		#define ACTION_DIM 4
	#endif
#endif

#define REACHING_TASK_FLAG 100
#define PICK_N_PLACE_TASK_FLAG 200

#define REACHING_FST_EE_POS_OFFSET 3
#define REACHING_DEST_POS_OFFSET (REACHING_FST_EE_POS_OFFSET + CART_DIM)
#define REACHING_SND_EE_POS_OFFSET (REACHING_DEST_POS_OFFSET + CART_DIM)
#define REACHING_TERMINAL_BIT_OFFSET (REACHING_SND_EE_POS_OFFSET + CART_DIM)
#define REACHING_REWARD_BIT_OFFSET (REACHING_TERMINAL_BIT_OFFSET + 1)

#define PNP_EE_POS_OFFSET 3
#define PNP_EE_STATE_OFFSET (PNP_EE_POS_OFFSET + CART_DIM)
#define PNP_FST_OBJ_POS_OFFSET (PNP_EE_STATE_OFFSET + 1)
#define PNP_HAS_OBJ_OFFSET (PNP_FST_OBJ_POS_OFFSET + CART_DIM)
#define PNP_DEST_POS_OFFSET (PNP_HAS_OBJ_OFFSET + 1)
#define PNP_SND_OBJ_POS_OFFSET (PNP_DEST_POS_OFFSET + CART_DIM)
#define PNP_TERMINAL_BIT_OFFSET (PNP_SND_OBJ_POS_OFFSET + CART_DIM)
#define PNP_REWARD_BIT_OFFSET (PNP_TERMINAL_BIT_OFFSET + 1)

#include "robot_reinforcement_learning/C/matrix_op.h"
#include "robot_reinforcement_learning/C/main.h"
#include "Sim.hpp"

extern "C" int initEnv(int act_dim, int task_flag);

extern "C" double rand_uniform(double low, double high);
// state: angles, target/coord, ee/coord, if succeed = 1*10 double
extern "C" matrix_t* resetState(int randAngle, int destPos, int state_dim, int act_dim);

// three angles to move about, NOT the angles to be moved to
// action::1*3 double
extern "C" matrix_t* step(matrix_t* action, int state_dim, int act_dim);

extern "C" void renderSteps(matrix_t** actions, int numOfActions);

extern "C" void closeEnv(int state_dim, int act_dim);

extern "C" matrix_t* random_action(int state_dim, int act_dim);

extern "C" matrix_t** collect_trace(char* path, int flag, char* norm_path, int task_flag);

extern "C" matrix_t* inverse_km(matrix_t* eePos);

#endif
