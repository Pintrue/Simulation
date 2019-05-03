#include "C_api.hpp"
#include "utils/Utils.hpp"
#include "robot_reinforcement_learning/C/utils.h"
#include <math.h>

#define JA0_L -M_PI/2
#define JA0_U M_PI/2
#define JA1_L 0.0
#define JA1_U 120.0/180.0*M_PI
#define JA2_L -M_PI/2
#define JA2_U 0.0
#define TIP_REACHED_RANGE 1.0

using namespace std;
using namespace KDL;

Sim sim;


int initEnv() {
	sim = Sim();

	// JntArray toJA = JntArray(NUM_OF_JOINTS);
	// toJA(0) = M_PI;			// Joint 1
	// toJA(1) = 0;			// Joint 2
	// toJA(2) = 0;			// Joint 3

	// // double toA[3] = {M_PI, 0.0, 0.0};
	// // sim.moveByJointAngles(toA, 10.0);

	// Frame eeFrame;
	// sim._km.jntsToCart(toJA, eeFrame);
	
	// // Provide this as API
	// sim._km._cartPose = eeFrame;
	// // print_frame(eeFrame);
	// print_frame(sim._km._cartPose);

	// cout << "Donezo" << endl;
	return 0;
}


matrix_t* resetState(int randAngle, int destPos) {
	matrix_t* fullState = new_matrix(1, FULL_STATE_NUM_COLS);
	double* data = fullState->data;
	
	// setting the initial segment angles
	if (randAngle == 1) {
		data[0] = rand_uniform(-M_PI/2, M_PI/2);
		data[1] = rand_uniform(0.0, 120.0/180.0*M_PI);
		data[2] = rand_uniform(-M_PI/2, 0);
	}

	// setting the current position of the end-effector
	JntArray angle = JntArray(NUM_OF_JOINTS);
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		angle(i) = data[i];
	}

	Frame eeFrame;
	sim._km.jntsToCart(angle, eeFrame);
	// print_frame(eeFrame);
	// cout << endl;

	for (int i = 0; i < CART_DIM; ++i) {
		data[i + 3] = eeFrame(i, 3);
	}
	

	// setting the target position for the end-effector
	if (destPos == 1) {
		// TODO: figure out the range of reach

		// a temp target for now
		data[6] = 0.0;
		data[7] = 0.0;
		data[8] = 10.0;
	}

	for (int i = 0; i < CART_DIM; ++i) {
		sim.target[i] = data[i + 6];
	}

	return fullState;
}


int ifInReach(double fullState[FULL_STATE_NUM_COLS]) {
	double diff = 0;

	for (int i = 3; i < 6; ++i) {
		double delta = fullState[i + 3] - fullState[i];
		diff += delta * delta;
	}
	// cout << sqrt(diff) << endl;
	return sqrt(diff) <= TIP_REACHED_RANGE;
}


matrix_t* step(matrix_t* action) {
	matrix_t* fullState = new_matrix(1, FULL_STATE_NUM_COLS);
	double* data = fullState->data;
	double* delta = action->data;
	JntArray ja = sim._km._jointAngles;

	// regulate the joint angle to within legal boundaries
	data[0] = min(max(ja(0)+delta[0], JA0_L), JA0_U);
	data[1] = min(max(ja(1)+delta[1], JA1_L), JA1_U);
	data[2] = min(max(ja(2)+delta[2], JA2_L), JA2_U);
	
	// calculate end-effector pose
	JntArray toJA = JntArray(NUM_OF_JOINTS);
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		toJA(i) = data[i];
	}

	Frame eeFrame;
	sim._km.jntsToCart(toJA, eeFrame);

	for (int i = 0; i < CART_DIM; ++i) {
		data[i + 3] = eeFrame(i, 3);
	}

	for (int i = 0; i < CART_DIM; ++i) {
		data[i + 6] = sim.target[i];
	}

	// set the terminal flag to 1, if within reach
	if (ifInReach(data)) {
		data[9] = 1;
	}

	return fullState;
}


// int main() {
// 	// initEnv();
// 	while (1) {
// 		matrix_t* full = resetState(1, 1);
// 		double* data = full->data;
// 		for (int i = 0; i < full->rows; ++i) {
// 			for (int j = 0; j < full->cols; ++j) {
// 				cout << *(data + i * full->cols + j) << " ";
// 			}
// 		}
// 		cout << endl;

// 		matrix_t* delta = new_matrix(1, 3);
// 		delta->data[0] = 0.1;
// 		delta->data[1] = 0;
// 		delta->data[2] = 0;
// 		matrix_t* newFull = step(delta);
// 		double* newData = newFull->data;
// 		for (int i = 0; i < newFull->rows; ++i) {
// 			for (int j = 0; j < newFull->cols; ++j) {
// 				cout << *(newData + i * newFull->cols + j) << " ";
// 			}
// 		}
// 		cout << endl;
// 		// cout << endl;
// 		// double test[10] = {0,0,0,1,2,3,1,2.5,3,0};
// 		// cout << ifInReach(test) << endl;
// 		break;
// 	}
// 	return 0;
// }