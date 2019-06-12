#include "Sim.hpp"
#include "utils/Utils.hpp"
#include <algorithm>
#include <iostream>
#include <stdio.h>

using namespace std;


/*
*   Initialize the simulation with the unit timestep
*   for the trajectory.
*/
Sim::Sim() {
	// set current joint angles to zero
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		_km._jointAngles[i] = 0;
	}

	_numOfSteps = 0;
	_hasObj = false;
	_eeState = false;
}


Sim::Sim(float ori[NUM_OF_JOINTS]) {
	// initialize kinematics and trajectory models
	_tjt.init(NUM_OF_JOINTS);
	
	// set current joint angles to zero
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		_km._jointAngles[i] = 0;
	}

	_numOfSteps = 0;
	_hasObj = false;
	_eeState = false;
}


float* Sim::getJointAngles() {
	return _km._jointAngles;
}


float* Sim::getEEPose() {
	return _km._cartPose;
}


/*
*   Move the each of the joints to the corresponding
*   angles specified in jointAngles[NUM_OF_JOINTS], in RADIUS.
*   Duration of the trajectory is specified by dura-
*   tion.
*/
void Sim::moveByJointAngles(float jointAngles[NUM_OF_JOINTS], float duration) {
	// JntArray toJA = JntArray(NUM_OF_JOINTS);
	float toJA[NUM_OF_JOINTS];
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		toJA[i] = jointAngles[i];
	}

	// plan the trajectory - calculate the coeficients
	_tjt.prepare(_km._jointAngles, toJA, duration);
	// JntArray interJA = JntArray(NUM_OF_JOINTS);
	float interJA[NUM_OF_JOINTS];
	cout << "t = " << _tjt.timeNow() << endl << endl;

	// keep looping if still in progress
	while (_tjt.nextTimeStep(_tjt.timeNow() + 1.0, interJA)) {
		cout << "t = " << _tjt.timeNow() << endl;
		cout << "Ongoing!" << endl << endl;
	}
	cout << "t = " << _tjt.timeNow() << endl;
	cout << "Finished." << endl << endl;
}


// int main() {
// 	// // origin where the robot is based
// 	float pos[NUM_OF_JOINTS] = {0.0, 0.0, 0.0};
	
// 	Sim sim(pos);

// 	// JntArray toJA = JntArray(NUM_OF_JOINTS);
// 	// toJA(0) = M_PI / 2;			// Joint 1
// 	// toJA(1) = 0;			        // Joint 2
// 	// toJA(2) = -(80.0/180.0*M_PI);// Joint 3

// 	// verification
// 	JntArray toJA = JntArray(NUM_OF_JOINTS);
// 	toJA(0) = M_PI / 2;			// Joint 1
// 	toJA(1) = 0;			// Joint 2
// 	toJA(2) = 0;			// Joint 3

// 	Frame eeFrame;
// 	sim._km.getPoseByJnts((toJA, eeFrame);
	
// 	// Provide this as API
// 	sim._km._cartPose = eeFrame;
// 	// printFrame(eeFrame);
// 	printFrame(sim._km._cartPose);

// 	cout << "Done" << endl;

// 	// // Function call from C
// 	// cout << endl << "Function call from C" << endl; 
// 	// function_call_test();
// 	_main();

// }