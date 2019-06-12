#ifndef SIM_HPP
#define SIM_HPP


#include "model/KinematicsModel.hpp"
#include "model/Trajectory.hpp"
#include "robot_reinforcement_learning/C/main.h"
#include "robot_reinforcement_learning/C/matrix_op.h"
#include "C_api.hpp"
#include "robot_reinforcement_learning/C/macros.h"

class Sim {
	public:
		Sim();				// default constructor
		Sim(float ori[3]); // constructor

		float* getJointAngles();
		float* getEEPose();
		void moveByJointAngles(float jointAngles[3], float duration);

		KinematicsModel _km;// kinematics model
		Trajectory _tjt;    // trajectory model

		float _initJA[NUM_OF_JOINTS];
		float _currentJA[NUM_OF_JOINTS];

		#ifdef RENDER
		matrix_t** _actions;	// denormalized actions from rl algo
		#endif
		int _numOfActions;

		float _target[CART_DIM];
		float _obj[CART_DIM];
		float _initObj[CART_DIM];
		bool _hasObj;
		bool _eeState;
		// float* _target;
		int _numOfSteps;

	private:
		

		// float _origin[3];
		// float _timeStep;
};

extern "C" matrix_t* function_call_test();

#endif