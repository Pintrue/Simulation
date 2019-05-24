#ifndef SIM_HPP
#define SIM_HPP


#include "model/KinematicsModel.hpp"
#include "model/Trajectory.hpp"
#include "robot_reinforcement_learning/C/main.h"
#include "robot_reinforcement_learning/C/matrix_op.h"
#include "C_api.hpp"

class Sim {
	public:
		Sim();				// default constructor
		Sim(double ori[3]); // constructor

		KDL::JntArray getJointAngles();
		KDL::Frame getEEPose();
		void moveByJointAngles(double jointAngles[3], double duration);

		KinematicsModel _km;// kinematics model
		Trajectory _tjt;    // trajectory model

		KDL::JntArray _initJA;
		double _currentJA[NUM_OF_JOINTS];

		double** _actions;
		int _numOfActions;

		double _target[CART_DIM];
		double _obj[CART_DIM];
		double _initObj[CART_DIM];
		bool _hasObj;
		bool _eeState;
		// double* _target;
		int _numOfSteps;

	private:
		

		// double _origin[3];
		// double _timeStep;
};

extern "C" matrix_t* function_call_test();

#endif