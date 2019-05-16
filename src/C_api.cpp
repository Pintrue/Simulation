#include "C_api.hpp"
#include "utils/Utils.hpp"
#include "robot_reinforcement_learning/C/utils.h"
#include "model/Constraints.hpp"

#ifdef RENDER
#include <QtWidgets/QApplication>
#include <QtWidgets/QDesktopWidget>
#include "GUI/QtMainWindow.hpp"
#include "GUI/QtWindow.hpp"
#endif

#include <math.h>

#define JA0_L -M_PI/2
#define JA0_U M_PI/2
#define JA1_L 0.0
#define JA1_U 120.0/180.0*M_PI
#define JA2_L -M_PI/2
#define JA2_U 0.0


using namespace std;
using namespace KDL;

#ifdef RENDER
QtWindow* window;
#endif
Sim sim;


int initEnv(int act_dim) {
	sim = Sim();
	return 0;
}


matrix_t* resetState(int randAngle, int destPos, int state_dim, int act_dim) {
	sim._numOfSteps = 0;	// reset number of steps executed

	matrix_t* fullState = new_matrix(1, FULL_STATE_NUM_COLS);
	double* data = fullState->data;
	
	// setting the initial segment angles
	if (randAngle == 1) {
		data[0] = rand_uniform(JA0_L, JA0_U);
		data[1] = rand_uniform(JA1_L, JA1_U);
		data[2] = rand_uniform(JA2_L, JA2_U);
	}

	// setting the current position of the end-effector
	JntArray angle = JntArray(NUM_OF_JOINTS);
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		angle(i) = data[i];
	}

	sim._initJA = angle;

	Frame eeFrame;
	sim._km.jntsToCart(angle, eeFrame);


	for (int i = 0; i < CART_DIM; ++i) {
		data[i + FST_EE_POS_OFFSET] = eeFrame.p(i);
		data[i + SND_EE_POS_OFFSET] = eeFrame.p(i);
		sim._target[i] = eeFrame.p(i);
	}
	// for (int i = 0; i < CART_DIM; ++i) {
	// 	sim._target[i] = data[i + FST_EE_POS_OFFSET];
	// }

	// setting the target position for the end-effector
	if (destPos == 1) {
		double dest[CART_DIM];

		// now destination is only considering GROUND-LEVEL workspace
		do {
			dest[0] = rand_uniform(-GROUND_OUT_RADIUS, GROUND_OUT_RADIUS);
			dest[1] = 0;
			dest[2] = rand_uniform(-GROUND_OUT_RADIUS, GROUND_OUT_RADIUS);
		} while (!inGroundLvlWorkspace(dest));

		for (int i = 0; i < CART_DIM; ++i) {
			data[i + DEST_POS_OFFSET] = dest[i];
		}
	}

	// for (int i = 0; i < CART_DIM; ++i) {
	// 	data[i + SND_EE_POS_OFFSET] = eeFrame.p(i);
	// }


	data[TERMINAL_BIT_OFFSET] = 0;	// terminal flag
	data[REWARD_BIT_OFFSET] = -1;	// reward bit

	return fullState;
}


int ifInReach(double fullState[FULL_STATE_NUM_COLS]) {
	double diff = 0;

	for (int i = 3; i < 6; ++i) {
		double delta = fullState[i + FST_EE_POS_OFFSET] - fullState[i];
		diff += delta * delta;
	}

	return sqrt(diff) <= TIP_REACHED_RANGE;
}


void setRewardBit(double fullState[FULL_STATE_NUM_COLS]) {
	fullState[REWARD_BIT_OFFSET] = ifInReach(fullState) ? 0 : -1;
}


matrix_t* step(matrix_t* action, int state_dim, int act_dim) {
	sim._numOfSteps += 1;	// increment the number of steps executed

	matrix_t* fullState = new_matrix(1, FULL_STATE_NUM_COLS);
	double* data = fullState->data;
	double* delta = action->data;
	JntArray ja = sim._km._jointAngles;

	// regulate the joint angle to within legal boundaries
	regulateJntAngles(ja, delta, data);
	
	// data[0] = min(max(ja(0)+delta[0], JA0_L), JA0_U);
	// data[1] = min(max(ja(1)+delta[1], JA1_L), JA1_U);
	// data[2] = min(max(ja(2)+delta[2], JA2_L), JA2_U);
	
	// calculate end-effector pose
	JntArray toJA = JntArray(NUM_OF_JOINTS);
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		toJA(i) = data[i];
	}

	Frame eeFrame;
	sim._km.jntsToCart(toJA, eeFrame);

	for (int i = 0; i < CART_DIM; ++i) {
		data[i + FST_EE_POS_OFFSET] = eeFrame.p(i);
		data[i + SND_EE_POS_OFFSET] = eeFrame.p(i);
	}

	for (int i = 0; i < CART_DIM; ++i) {
		data[i + DEST_POS_OFFSET] = sim._target[i];
	}


	// for (int i = 0; i < CART_DIM; ++i) {
	// 	data[i + 6] = eeFrame.p(i);
	// }
	// this simulation set-up should terminate after 50 steps
	if (sim._numOfSteps >= 50)
		data[TERMINAL_BIT_OFFSET] = 1;

	// set the reward function
	setRewardBit(data);

	return fullState;
}


#ifdef RENDER
void renderSteps(matrix_t** actions, int numOfActions) {
	sim._numOfActions = numOfActions;
	for (int i = 0; i < numOfActions; ++i) {
		sim._actions[i] = actions[i]->data;
	}

	char arg0[] = "Program Name";
    char arg1[] = "pedantically";
    char arg2[] = "starting";
    char* argv[] = {&arg0[0], &arg1[0], &arg2[0], NULL};
    int argc   = (int)(sizeof(argv) / sizeof(argv[0])) - 1;
	QApplication app(argc, argv);
	QtMainWindow mainWindow;
	window = new QtWindow(&mainWindow);
	window->getGLWidgets()->setSim(sim);
	window->resize(window->sizeHint());
	int desktopArea = QApplication::desktop()->width() * QApplication::desktop()->height();
    int widgetArea = window->width() * window->height();

	emit window->getGLWidgets()->moveByActionPath();

	if (((float)widgetArea / (float)desktopArea) < 0.75f)
        window->show();
    else
        window->showMaximized();
    app.exec();
}
#endif


void closeEnv(int state_dim, int act_dim) {
	#ifdef RENDER
	delete window;
	#endif
	return;
}


matrix_t* random_action(int state_dim, int act_dim) {
	matrix_t* ret = new_matrix(1, act_dim);
	double* data = ret->data;
	double toJA[NUM_OF_JOINTS];
	toJA[0] = rand_uniform(JA0_L, JA0_U);
	toJA[1] = rand_uniform(JA1_L, JA1_U);
	toJA[2] = rand_uniform(JA2_L, JA2_U);

	JntArray ja = sim._km._jointAngles;
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		data[i] = toJA[i] - ja(i);
	}

	return ret;
}


int main() {
	initEnv(0);
	while (1) {
		cout << "Initialize all states" << endl;
		matrix_t* full = resetState(1, 1, 0, 0);
		// printJntArray(sim._initJA);
		double* data = full->data;
		for (int i = 0; i < full->rows; ++i) {
			for (int j = 0; j < full->cols; ++j) {
				cout << *(data + i * full->cols + j) << " ";
			}
		}
		cout << endl;

		matrix_t* steps[3];
		steps[0] = new_matrix(1, ACTION_DIM);
		steps[0]->data[1] = 0.5;
		steps[1] = new_matrix(1, ACTION_DIM);
		steps[1]->data[1] = 0.5;
		steps[2] = new_matrix(1, ACTION_DIM);
		steps[2]->data[1] = 0.5;

		renderSteps(steps, 3);
		
		// setRewardBit(data);
		// for (int i = 0; i < full->rows; ++i) {
		// 	for (int j = 0; j < full->cols; ++j) {
		// 		cout << *(data + i * full->cols + j) << " ";
		// 	}
		// }

		// cout << "Make one step" << endl;
		// matrix_t* delta = new_matrix(1, 3);
		// delta->data[0] = 0.1;
		// delta->data[1] = 0;
		// delta->data[2] = 0;
		// matrix_t* newFull = step(delta, 0, 0);
		// double* newData = newFull->data;
		// for (int i = 0; i < newFull->rows; ++i) {
		// 	for (int j = 0; j < newFull->cols; ++j) {
		// 		cout << *(newData + i * newFull->cols + j) << " ";
		// 	}
		// }
		// cout << endl;
		// double test[10] = {0,0,0,1,2,3,1,2.5,3,0};
		// cout << ifInReach(test) << endl;

		break;
	}
	_main();
	return 0;
}