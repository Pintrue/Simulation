#include "Sim.hpp"
#include "utils/Utils.hpp"
#include <algorithm>

using namespace std;
using namespace KDL;


/*
*   Initialize the simulation with the unit timestep
*   for the trajectory.
*/
Sim::Sim() {
	double pos[NUM_OF_JOINTS] = {0.0, 0.0, 0.0};
	_km.init(pos);
	// _tjt.init(NUM_OF_JOINTS);
	
	// set current joint angles to zero
	JntArray fromJA = JntArray(NUM_OF_JOINTS);
	_km._jointAngles = fromJA;
	SetToZero(_km._jointAngles);

	_numOfSteps = 0;
}


Sim::Sim(double ori[NUM_OF_JOINTS]) {
	// copy(ori, ori + NUM_OF_JOINTS, _origin);

	// initialize kinematics and trajectory models
	_km.init(ori);
	_tjt.init(NUM_OF_JOINTS);
	
	// set current joint angles to zero
	JntArray fromJA = JntArray(NUM_OF_JOINTS);
	_km._jointAngles = fromJA;
	SetToZero(_km._jointAngles);

	_numOfSteps = 0;
}


JntArray Sim::getJointAngles() {
	return _km._jointAngles;
}


Frame Sim::getEEPose() {
	return _km._cartPose;
}


/*
*   Move the each of the joints to the corresponding
*   angles specified in jointAngles[NUM_OF_JOINTS], in RADIUS.
*   Duration of the trajectory is specified by dura-
*   tion.
*/
void Sim::moveByJointAngles(double jointAngles[NUM_OF_JOINTS], double duration) {
	JntArray toJA = JntArray(NUM_OF_JOINTS);
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		toJA(i) = jointAngles[i];
	}

	// plan the trajectory - calculate the coeficients
	_tjt.prepare(_km._jointAngles, toJA, duration);
	JntArray interJA = JntArray(NUM_OF_JOINTS);

	cout << "t = " << _tjt.timeNow() << endl << endl;

	// keep looping if still in progress
	while (_tjt.nextTimeStep(_tjt.timeNow() + 1.0, interJA)) {
		cout << "t = " << _tjt.timeNow() << endl;
		cout << "Ongoing!" << endl << endl;
	}
	cout << "t = " << _tjt.timeNow() << endl;
	cout << "Finished." << endl << endl;

	// _km._jointAngles = toJA; <- carried out by fwdKmt(...)
}


// int main() {
// 	// // origin where the robot is based
// 	double pos[NUM_OF_JOINTS] = {0.0, 0.0, 0.0};
	
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
// 	sim._km.fwdKmt((toJA, eeFrame);
	
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