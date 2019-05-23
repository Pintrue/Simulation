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
#include "model/FwdKM.h"

#define JA0_L -M_PI/2
#define JA0_U M_PI/2
#define JA1_L 0.0
#define JA1_U 120.0/180.0*M_PI
#define JA2_L -M_PI/2
#define JA2_U 0.0

#define ACTION_BOUND_UPPER 0.0872664626
#define ACTION_BOUND_LOWER -0.0872664626
#define OBJ_HEIGHT 0.5

using namespace std;
using namespace KDL;

#ifdef RENDER
QtWindow* window;
#endif
int taskFlag;
static Sim sim;


int initEnv(int act_dim, int task_flag) {
	if (task_flag == REACHING_TASK_FLAG) {
		/* Init. training for reaching task */
		sim = Sim();
	} else if (task_flag == PICK_N_PLACE_TASK_FLAG) {
		/* Init. training for pick and place task */
		sim = Sim();
	} else {
		cout << "ERROR: Unrecognized flag encountered." << endl;
		cout << "Halt now." << endl;
		exit(EXIT_FAILURE);
	}
	
	taskFlag = task_flag;
	return 0;
}


matrix_t* resetStateReaching(int randAngle, int destPos, int state_dim, int act_dim) {
	/* Nothing needs to be done rn.*/
	cout << "Reaching full state dim: " << FULL_STATE_NUM_COLS << endl;
	matrix_t* ret = new_matrix(1, FULL_STATE_NUM_COLS);
	double* data = ret->data;

	sim._numOfSteps = 0;	// reset number of steps executed
	
	// setting the initial segment angles
	if (randAngle == 1) {
		data[0] = rand_uniform(JA0_L, JA0_U);
		data[1] = rand_uniform(JA1_L, JA1_U);
		data[2] = rand_uniform(JA2_L, JA2_U);
	} else {
		for (int i = 0; i < NUM_OF_JOINTS; ++i) {
			data[i] = 0;
		}
	}

	/* setting the current position of the end-effector */

	// TESTING VARIABLE, delete later.
	double eePos[6];
	
	JntArray angle = JntArray(NUM_OF_JOINTS);
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		angle(i) = data[i];
		sim._currentJA[i] = data[i];
	}

	sim._initJA = angle;
	
	/* USED to make user of kdl, get rid of it now*/

	initKM();
	getEEPoseByJnts(sim._currentJA, eePos);


	for (int i = 0; i < CART_DIM; ++i) {
		// if (eePos[i] + 0.006 < eeFrame.p(i) || eePos[i] - 0.006 > eeFrame.p(i)) {
		// 	for (int i = 0; i < 10; ++i) {
		// 		cout << "Diff!" << endl;
		// 	}
		// 	cout << "Yours for " << i << " is " << eePos[i] << endl;
		// 	cout << "KDL for " << i << " is " << eeFrame.p[i] << endl;
		// 	exit(0);
		// }

		// data[i + FST_EE_POS_OFFSET] = eeFrame.p(i);
		// data[i + SND_EE_POS_OFFSET] = eeFrame.p(i);

		data[i + REACHING_FST_EE_POS_OFFSET] = eePos[i];
		data[i + REACHING_SND_EE_POS_OFFSET] = eePos[i];
	}

	// setting the target position for the end-effector
	if (destPos == 1) {
		matrix_t* destPos = new_matrix(1, CART_DIM);
		double* dest = destPos->data;

		dest[0] = rand_uniform(-10.5, 10.5);
		dest[1] = 0;
		dest[2] = rand_uniform(12.5, 22.5);

		cout << "This is the target position" << endl;
		print_matrix(destPos, 1);

		for (int i = 0; i < CART_DIM; ++i) {
			data[i + REACHING_DEST_POS_OFFSET] = dest[i];
			sim._target[i] = dest[i];
		}
		free_matrix(destPos);
	} else {
		data[0 + REACHING_DEST_POS_OFFSET] = 0;
		data[1 + REACHING_DEST_POS_OFFSET] = 0;
		data[2 + REACHING_DEST_POS_OFFSET] = 17.33;
	}

	data[REACHING_TERMINAL_BIT_OFFSET] = 0;	// terminal flag
	data[REACHING_REWARD_BIT_OFFSET] = -1;	// reward bit

	return ret;
}

matrix_t* resetStatePnP(int randAngle, int destPos, int state_dim, int act_dim) {
	cout << "PnP full state dim: " << FULL_STATE_NUM_COLS << endl;
	matrix_t* ret = new_matrix(1, FULL_STATE_NUM_COLS);
	double* data = ret->data;

	sim._numOfSteps = 0;	// reset number of steps executed
	
	// setting the initial segment angles
	if (randAngle == 1) {
		data[0] = rand_uniform(JA0_L, JA0_U);
		data[1] = rand_uniform(JA1_L, JA1_U);
		data[2] = rand_uniform(JA2_L, JA2_U);
	} else {
		for (int i = 0; i < NUM_OF_JOINTS; ++i) {
			data[i] = 0;
		}
	}

	/* setting the current position of the end-effector */
	double eePos[6];
	
	JntArray angle = JntArray(NUM_OF_JOINTS);
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		angle(i) = data[i];
		sim._currentJA[i] = data[i];
	}

	sim._initJA = angle;
	
	/* USED to make user of kdl, get rid of it now*/

	initKM();
	getEEPoseByJnts(sim._currentJA, eePos);


	for (int i = 0; i < CART_DIM; ++i) {
		data[i + PNP_EE_POS_OFFSET] = eePos[i];
	}

	/**	data[...] = 0 -> Turn magnet off
	 * 	data[...] = 1 -> Turn magnet on

	 *  Magnet not turned on at the beginning.
	 * 
	 **/
	data[PNP_EE_STATE_OFFSET] = 0;

	/* setting the object position for the PICK */
	matrix_t* objPos = new_matrix(1, CART_DIM);
	double* obj = objPos->data;

	obj[0] = rand_uniform(-10.5, 10.5);
	obj[1] = OBJ_HEIGHT;
	obj[2] = rand_uniform(12.5, 22.5);
	cout << "This is the target position" << endl;
	print_matrix(objPos, 1);

	for (int i = 0; i < CART_DIM; ++i) {
		data[i + PNP_FST_OBJ_POS_OFFSET] = obj[i];
		data[i + PNP_SND_OBJ_POS_OFFSET] = obj[i]; 
		sim._obj[i] = obj[i];
	}
	free_matrix(objPos);


	// TODO: if has object now?
	/* setting if the EE has the object now */
	data[PNP_HAS_OBJ_OFFSET] = 0;


	/* setting the target position for the PLACE */
	if (destPos == 1) {
		matrix_t* destPos = new_matrix(1, CART_DIM);
		double* dest = destPos->data;

		dest[0] = rand_uniform(-10.5, 10.5);
		dest[1] = 0;
		dest[2] = rand_uniform(12.5, 22.5);

		cout << "This is the target position" << endl;
		print_matrix(destPos, 1);

		for (int i = 0; i < CART_DIM; ++i) {
			data[i + PNP_DEST_POS_OFFSET] = dest[i];
			sim._target[i] = dest[i];
		}
		free_matrix(destPos);
	} else {
		data[0 + PNP_DEST_POS_OFFSET] = 0;
		data[1 + PNP_DEST_POS_OFFSET] = 0;
		data[2 + PNP_DEST_POS_OFFSET] = 17.33;
	}

	data[PNP_TERMINAL_BIT_OFFSET] = 0;	// terminal flag
	data[PNP_REWARD_BIT_OFFSET] = -1;	// reward bit

	return ret;
}


matrix_t* resetState(int randAngle, int destPos, int state_dim, int act_dim) {
	matrix_t* ret;

	if (taskFlag == REACHING_TASK_FLAG) {
		cout << "Reaching task flag set." << endl;
		ret = resetStateReaching(randAngle, destPos, state_dim, act_dim);
	} else if (taskFlag == PICK_N_PLACE_TASK_FLAG) {
		cout << "Pick n place task flag set." << endl;
		ret = resetStatePnP(randAngle, destPos, state_dim, act_dim);
	} else {
		/* SHOULD NOT have reached here. */
		cout << "ERROR: Unrecognized flag encountered at resetState()." << endl;
		cout << "Halt now." << endl;
		exit(EXIT_FAILURE);
	}

	return ret;
}


int ifInReach(double fullState[FULL_STATE_NUM_COLS]) {
	double diff = 0;

	for (int i = 3; i < 6; ++i) {
		double delta = fullState[i + REACHING_FST_EE_POS_OFFSET] - fullState[i];
		diff += delta * delta;
	}

	return sqrt(diff) <= TIP_REACHED_RANGE;
}


int ifHadObj(double fullState[FULL_STATE_NUM_COLS]) {
	double diff = 0;

	for (int i = 0; i < CART_DIM; ++i) {
		double delta = fullState[i + PNP_EE_POS_OFFSET] - fullState[i + PNP_FST_OBJ_POS_OFFSET];
		diff += delta * delta;
	}

	return sqrt(diff) <= OBJ_ATTACHED_RANGE;
}


int ifObjNearDest(double fullState[FULL_STATE_NUM_COLS]) {
	double diff = 0;

	for (int i = 0; i < CART_DIM; ++i) {
		double delta = fullState[i + PNP_FST_OBJ_POS_OFFSET] - fullState[i + PNP_DEST_POS_OFFSET];
		diff += delta * delta;
	}

	return sqrt(diff) <= OBJ_NEAR_DEST_RANGE;
}

void setReachingRewardBit(double fullState[FULL_STATE_NUM_COLS]) {
	fullState[REACHING_REWARD_BIT_OFFSET] = ifInReach(fullState) ? 0 : -1;
}


void setPnPRewardBit(double fullState[FULL_STATE_NUM_COLS]) {
	fullState[PNP_REWARD_BIT_OFFSET] = ifObjNearDest(fullState) ? 0 : -1;
}


matrix_t* denormalize_action(matrix_t* action) {
	matrix_t* ret = new_matrix(action->rows, action->cols);
	// double upper = 0.0872664626;
	// double lower = -0.0872664626;
	double d1 = (double)(action->data[0] + 1) / (double)2 * (ACTION_BOUND_UPPER - ACTION_BOUND_LOWER) + ACTION_BOUND_LOWER;
	double d2 = (double)(action->data[1] + 1) / (double)2 * (ACTION_BOUND_UPPER - ACTION_BOUND_LOWER) + ACTION_BOUND_LOWER;
	double d3 = (double)(action->data[2] + 1) / (double)2 * (ACTION_BOUND_UPPER - ACTION_BOUND_LOWER) + ACTION_BOUND_LOWER;
	ret->data[0] = d1;
	ret->data[1] = d2;
	ret->data[2] = d3;
	return ret;
}


matrix_t* normalize_action(matrix_t* action) {
	matrix_t* ret = new_matrix(action->rows, action->cols);
	ret->data[0] = (double) (action->data[0] - JA0_L) / (JA0_U - JA0_L) * 2 - 1;
	ret->data[1] = (double) (action->data[1] - JA1_L) / (JA1_U - JA1_L) * 2 - 1;
	ret->data[2] = (double) (action->data[2] - JA2_L) / (JA2_U - JA2_L) * 2 - 1;
	return ret;
}


matrix_t* stepReaching(matrix_t* action, int state_dim, int act_dim) {
	matrix_t* denormed_matrix = denormalize_action(action);
	sim._numOfSteps += 1;	// increment the number of steps executed

	matrix_t* fullState = new_matrix(1, FULL_STATE_NUM_COLS);
	double* data = fullState->data;
	double* delta = denormed_matrix->data;
	JntArray ja = JntArray(NUM_OF_JOINTS);//sim._km._jointAngles;
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		ja(i) = sim._currentJA[i];
	}

	/* regulate the joint angle to within legal boundaries */
	regulateJntAngles(ja, delta, data);
	
	/* calculate end-effector pose */

	// JntArray toJA = JntArray(NUM_OF_JOINTS);
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		// toJA(i) = data[i];
		sim._currentJA[i] = data[i];
	}

	// Frame eeFrame;
	// sim._km.getPoseByJnts(toJA, eeFrame);

	double eePos[6];

	initKM();
	getEEPoseByJnts(sim._currentJA, eePos);

	for (int i = 0; i < CART_DIM; ++i) {
		data[i + REACHING_FST_EE_POS_OFFSET] = eePos[i];
		data[i + REACHING_SND_EE_POS_OFFSET] = eePos[i];
	}

	for (int i = 0; i < CART_DIM; ++i) {
		data[i + REACHING_DEST_POS_OFFSET] = sim._target[i];
	}

	// this simulation set-up should terminate after 50 steps
	if (sim._numOfSteps >= 50)
		data[REACHING_TERMINAL_BIT_OFFSET] = 1;

	// set the reward function
	setReachingRewardBit(data);
	free_matrix(denormed_matrix);
	return fullState;
}


matrix_t* stepPnP(matrix_t* action, int state_dim, int act_dim) {
	matrix_t* jaAction = new_matrix(1, 3);
	jaAction->data[0] = action->data[0];
	jaAction->data[1] = action->data[1];
	jaAction->data[2] = action->data[2];

	matrix_t* denormed_ja_matrix = denormalize_action(jaAction);
	matrix_t* denormed_matrix = new_matrix(1, 4);
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		denormed_matrix->data[i] = denormed_ja_matrix->data[i];
	}
	free_matrix(jaAction);
	free_matrix(denormed_ja_matrix);
	denormed_matrix->data[3] = action->data[3];

	sim._numOfSteps += 1;	// increment the number of steps executed

	matrix_t* fullState = new_matrix(1, FULL_STATE_NUM_COLS);
	double* data = fullState->data;
	double* delta = denormed_matrix->data;

	JntArray ja = JntArray(NUM_OF_JOINTS);
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		ja(i) = sim._currentJA[i];
	}

	/* regulate the joint angle to within legal boundaries */
	regulateJntAngles(ja, delta, data);
	
	/* calculate end-effector pose */
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		sim._currentJA[i] = data[i];
	}

	double eePos[6];

	initKM();
	getEEPoseByJnts(sim._currentJA, eePos);

	for (int i = 0; i < CART_DIM; ++i) {
		data[i + PNP_EE_POS_OFFSET] = eePos[i];
	}

	/* setting the end-effector magnet current */
	data[PNP_EE_STATE_OFFSET] = action->data[3];
	
	/* setting if has the object */
	if (sim._hasObj && data[PNP_EE_STATE_OFFSET] == 1) {
		/* Object position changes with end-effector if, in 
		 * the last step, the object is already picked and 
		 * the EE state at this step is on.
		 **/
		for (int i = 0; i < CART_DIM; ++i) {
			sim._obj[i] = data[i + PNP_EE_POS_OFFSET];
		}
		data[PNP_HAS_OBJ_OFFSET] = 1;
		sim._eeState = true;
	} else if (sim._hasObj && data[PNP_EE_STATE_OFFSET] == 0) {
		/* magnet current off so drop the obj */
		sim._obj[1] = OBJ_HEIGHT;
		sim._hasObj = false;
		data[PNP_HAS_OBJ_OFFSET] = 0;
	} else if ((!sim._hasObj) && data[PNP_EE_STATE_OFFSET] == 1) {
		/* setting if magnet has the object when it does not already have
		 * in the last time step, by checking if the EE with magnet current
		 * on is in certain range to the object.
		 **/
		if (ifHadObj(data)) {
			data[PNP_HAS_OBJ_OFFSET] = 1;
			for (int i = 0; i < CART_DIM; ++i) {
				sim._obj[i] = data[i + PNP_EE_POS_OFFSET];
			}
			sim._hasObj = true;
			data[PNP_HAS_OBJ_OFFSET] = 1;
		}
		sim._eeState = true;
	}

	/* set the position of the object*/
	for (int i = 0; i < CART_DIM; ++i) {
		data[i + PNP_FST_OBJ_POS_OFFSET] = sim._obj[i];
		data[i + PNP_SND_OBJ_POS_OFFSET] = sim._obj[i];
	}

	for (int i = 0; i < CART_DIM; ++i) {
		data[i + PNP_DEST_POS_OFFSET] = sim._target[i];
	}

	if (sim._numOfSteps >= 50) {
		data[PNP_TERMINAL_BIT_OFFSET] = 1;
	}

	setPnPRewardBit(data);
	free_matrix(denormed_matrix);
	return fullState;
}


matrix_t* step(matrix_t* action, int state_dim, int act_dim) {
	matrix_t* ret;

	if (taskFlag == REACHING_TASK_FLAG) {
		cout << "Reaching task flag set." << endl;
		ret = stepReaching(action, state_dim, act_dim);
	} else if (taskFlag == PICK_N_PLACE_TASK_FLAG) {
		cout << "Pick n place task flag set." << endl;
		ret = stepPnP(action, state_dim, act_dim);
	} else {
		/* SHOULD NOT have reached here. */
		cout << "ERROR: Unrecognized flag encountered at resetState()." << endl;
		cout << "Halt now." << endl;
		exit(EXIT_FAILURE);
	}

	return ret;
}

matrix_t* inverse_km(matrix_t* eePos) {
	// Frame eeFrame;
	double pose[POSE_DIM];

	for (int i = 0; i < CART_DIM; ++i) {
		// eeFrame.p(i) = eePos->data[i];
		pose[i] = eePos->data[i];
	}

	JntArray ja = JntArray(NUM_OF_JOINTS);
	sim._km.getJntsByPose(pose, ja);

	matrix_t* ret = new_matrix(1, NUM_OF_JOINTS);
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		ret->data[i] = ja(i);
	}
	return ret;
}


#ifdef RENDER
void renderSteps(matrix_t** actions, int numOfActions) {
	sim._actions = (double**) calloc(numOfActions, sizeof(double*));
	sim._numOfActions = numOfActions;
	for (int i = 0; i < numOfActions; ++i) {
		matrix_t* denormedAction = denormalize_action(actions[i]);
		sim._actions[i] = denormedAction->data;
	}

	char arg0[] = "Program Name";
    char arg1[] = "pedantically";
    char arg2[] = "starting";
    char* argv[] = {&arg0[0], &arg1[0], &arg2[0], NULL};
    int argc   = (int)(sizeof(argv) / sizeof(argv[0])) - 1;
	QApplication app(argc, argv);
	QtMainWindow mainWindow;
	window = new QtWindow(&mainWindow);
	// cout << "At renderSteps()" << endl;
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
	finishKM();
	return;
}


matrix_t* reachingRandomAction(int state_dim, int act_dim) {
	matrix_t* ret = new_matrix(1, act_dim);
	double* data = ret->data;
	double toJA[NUM_OF_JOINTS];
	toJA[0] = rand_uniform(JA0_L, JA0_U);
	toJA[1] = rand_uniform(JA1_L, JA1_U);
	toJA[2] = rand_uniform(JA2_L, JA2_U);

	// JntArray ja = sim._km._jointAngles;
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		// data[i] = toJA[i] - ja(i);
		data[i] = toJA[i] - sim._currentJA[i];
	}

	matrix_t* norm_ret = normalize_action(ret);
	free_matrix(ret);
	return norm_ret;
}


matrix_t* pnpRandomAction(int state_dim, int act_dim) {
	matrix_t* ret = new_matrix(1, act_dim);
	double* data = ret->data;

	matrix_t* jaActions = reachingRandomAction(0, act_dim - 1);
	for (int i = 0; i < act_dim - 1; ++i) {
		data[i] = jaActions->data[i];
	}
	data[3] = (sim._eeState) ? 1 : 0;
	free_matrix(jaActions);
	return ret;
}


matrix_t* random_action(int state_dim, int act_dim) {
	matrix_t* ret;

	if (taskFlag == REACHING_TASK_FLAG) {
		cout << "Reaching task flag set." << endl;
		ret = reachingRandomAction(state_dim, act_dim);
	} else if (taskFlag == PICK_N_PLACE_TASK_FLAG) {
		cout << "Pick n place task flag set." << endl;
		ret = pnpRandomAction(state_dim, act_dim);
	} else {
		/* SHOULD NOT have reached here. */
		cout << "ERROR: Unrecognized flag encountered at resetState()." << endl;
		cout << "Halt now." << endl;
		exit(EXIT_FAILURE);
	}

	return ret;
}


// int main() {
	// initEnv(0, 200);
	// while (1) {
	// 	cout << "Initialize all states" << endl;
	// 	matrix_t* full = resetState(1, 1, 0, 0);
		
	// 	double* data = full->data;
	// 	for (int i = 0; i < full->rows; ++i) {
	// 		for (int j = 0; j < full->cols; ++j) {
	// 			cout << *(data + i * full->cols + j) << " ";
	// 		}
	// 	}
	// 	cout << endl;
	// 	break;
	// }

	// 	matrix_t* steps[3];
	// 	steps[0] = new_matrix(1, ACTION_DIM);
	// 	steps[0]->data[1] = 0.5;
	// 	steps[1] = new_matrix(1, ACTION_DIM);
	// 	steps[1]->data[1] = 0.5;
	// 	steps[2] = new_matrix(1, ACTION_DIM);
	// 	steps[2]->data[1] = 0.5;

	// 	renderSteps(steps, 3);
		
	// 	// setReachingRewardBit(data);
	// 	// for (int i = 0; i < full->rows; ++i) {
	// 	// 	for (int j = 0; j < full->cols; ++j) {
	// 	// 		cout << *(data + i * full->cols + j) << " ";
	// 	// 	}
	// 	// }

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

	// 	// cout << endl;
	// 	// double test[10] = {0,0,0,1,2,3,1,2.5,3,0};
	// 	// cout << ifInReach(test) << endl;

	// 	break;
	// }
	// _main();

	// double ori[3] = {0.0, 0.0, 0.0};
	// KinematicsModel fwdKM = KinematicsModel();
	// fwdKM.init(ori);

	// KinematicsModel invKM = KinematicsModel();
	// invKM.init(ori);

	// JntArray toJA = JntArray(NUM_OF_JOINTS);
	// toJA(0) = M_PI / 2.0 / 3.0; // 30 deg
	// toJA(1) = M_PI * 0.6111; // 110 deg
	// toJA(2) = -M_PI * 0.3889; // 70 deg

	// printJntArray(toJA);
	// // Frame eeFrame;
	// double eePos[6];
	// fwdKM.getPoseByJnts(toJA, eePos);
	// // printFrame(eeFrame);
	// printPose(eePos);
	// cout << endl;

	// // Frame invEEFrame;
	// // invEEFrame.p(0) = eeFrame.p(0);
	// // invEEFrame.p(1) = eeFrame.p(1);
	// // invEEFrame.p(2) = eeFrame.p(2);
	// eePos[3] = 0;
	// eePos[4] = 0;
	// eePos[5] = 0;
	// JntArray invToJA = JntArray(NUM_OF_JOINTS);
	// invKM.getJntsByPose(eePos, invToJA);
	// printJntArray(invToJA);
	// // printFrame(invEEFrame);
	// printPose(eePos);
	// cout << endl;

	// KinematicsModel invKM1 = KinematicsModel();
	// invKM1.init(ori);
	// invKM1._jointAngles(0) = 0.5236;
	// double eePos1[6];
	// eePos1[0] = eePos[0];
	// eePos1[1] = eePos[1];
	// eePos1[2] = eePos[2];
	// // Frame invEEFrame1;
	// // invEEFrame1.p(0) = eeFrame.p(0);
	// // invEEFrame1.p(1) = eeFrame.p(1);
	// // invEEFrame1.p(2) = eeFrame.p(2);
	// JntArray invToJA1 = JntArray(NUM_OF_JOINTS);
	// invKM1.getJntsByPose(eePos1, invToJA1);
	// printJntArray(invToJA1);
	// printPose(eePos1);

	// matrix_t* test = new_matrix(1,3);
	// test->data[0] = 13.42;
	// test->data[1] = 7.604;
	// test->data[2] = 23.24;

	// matrix_t* ret = inverse_km(test);
	// print_matrix(ret, 1);
	// free_matrix(test);
	// free_matrix(ret);

	// while (1) {
	// 	matrix_t** mat = collect_trace((char*)"DDPG_ACTOR_SIM_NORM.model", 1, (char*)"DDPG_NORM_SIM_NORM.norm");
	// 	for (int i = 0; i < 50; ++i) {
	// 		print_matrix(denormalize_action(mat[i]), 1);
	// 	}
	// 	renderSteps(mat, 50);
	// }
	// return 0;
// }