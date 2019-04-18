#include <stdio.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include "KinematicsModel.hpp"

using namespace std;
using namespace KDL;

KinematicsModel::KinematicsModel() {}

void KinematicsModel::init(double origin[3]) {
    // Construct segments: links of the arm

    // Origin
    _kdlChain.addSegment(Segment(Joint(Joint::None), Frame(
        Vector(origin[0], origin[1], origin[2]))));
    // Base joint
    _kdlChain.addSegment(Segment(Joint(Joint::RotY), Frame(
        Vector(0.0, 4.20, 0.0))));
    // Shoulder joint
	_kdlChain.addSegment(Segment(Joint(Joint::RotX),Frame(
        Vector(0.0, 3.30, 5.0))));
    // Elbow joint
	_kdlChain.addSegment(Segment(Joint(Joint::None),Frame(
        Vector(0.0, 2.0, -10.57))));
    // End-effector
	_kdlChain.addSegment(Segment(Joint(Joint::RotX),Frame(
        Vector(0.0, 3.50, 18.4))));

    // Initialize 
    unsigned int numJoints = _kdlChain.getNrOfJoints();
    _jointAngles = JntArray(numJoints);
}


bool KinematicsModel::jntsToCart(const JntArray& jointAngles, Frame& eeFrame) {
    ChainFkSolverPos_recursive fKSolver = ChainFkSolverPos_recursive(_kdlChain);

    if (fKSolver.JntToCart(jointAngles, eeFrame) >= 0) {
        return true;
    } else {
        return false;
    }
}

// void print_frame(const Frame &eeFrame) {// Print the frame
// 	for (int i = 0; i < 4; i++){
// 		for (int j = 0; j < 4; j++) {
// 			double a = eeFrame(i, j);
// 			if (a < 0.0001 && a > -0.001) {
// 				a = 0.0;
// 			}
// 			cout << setprecision(4) << a << "\t\t";
// 		}
// 		cout << endl;
// 	}
// }

// int main() {
//     KinematicsModel km;

//     double pos[3] = {0.0, 0.0, 0.0};
//     km.init(pos);

//     JntArray jointAngles = JntArray(3);
// 	jointAngles(0) = M_PI / 2;			// Joint 1
// 	jointAngles(1) = 0;			// Joint 2
// 	jointAngles(2) = -(80.0/180.0*M_PI);			// Joint 3

//     Frame eeFrame;
//     km.jntsToCart(jointAngles, eeFrame);
//     print_frame(eeFrame);
// }

