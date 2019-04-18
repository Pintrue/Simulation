#include <iostream>
#include <iomanip>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

using namespace std;
using namespace KDL;

void print_end_effector(const Frame &eeFrame) {// Print the frame
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++) {
			double a = eeFrame(i, j);
			if (a < 0.0001 && a > -0.001) {
				a = 0.0;
			}
			cout << setprecision(4) << a << "\t\t";
		}
		cout << endl;
	}
}

int main() {
    Chain kdlChain; //= Chain();

	// Construct segments: links of the arm
	kdlChain.addSegment( Segment( Joint(Joint::None),Frame(Vector(0.0, 0.0, 0.0)) ));
	// kdlChain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0, 4.75, 0.0))));
	kdlChain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0, 4.20, 0.0))));
	// kdlChain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0, 4.47, 3.05))));
	kdlChain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0, 3.30, 5.0))));
	// kdlChain.addSegment(Segment(Joint(Joint::None),Frame(Vector(0.0, 1.43, -10.57))));
	kdlChain.addSegment(Segment(Joint(Joint::None),Frame(Vector(0.0, 2.0, -10.57))));
	// kdlChain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0, 7.70, 16.2))));
	kdlChain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0, 3.50, 18.4))));

	// Joint Angles
	unsigned int nj = kdlChain.getNrOfJoints();
	cout << "Number of joints is " << nj << endl;
	if (nj != 3) {
		printf("%s \n","Error: Number of joints does not match.");
		return -1;
	}

	JntArray jointAngles = JntArray(nj);
	jointAngles(0) = M_PI / 2;			// Joint 1
	jointAngles(1) = 0;			// Joint 2
	jointAngles(2) = -(80.0/180.0*M_PI);			// Joint 3
	
	// Perform Forward Kinematics
	ChainFkSolverPos_recursive FKSolver = ChainFkSolverPos_recursive(kdlChain);

	Frame eeFrame;
	int fk_status;
	// fk_status = FKSolver.JntToCart(jointAngles, eeFrame);

	// print_end_effector(eeFrame);

	// JntArray jointAngles1 = JntArray(nj);
	// jointAngles1(0) = M_PI / 2;			// Joint 1
	// jointAngles1(1) = 0;			// Joint 2
	// jointAngles1(2) = -(80.0/180.0*M_PI);

	// fk_status = FKSolver.JntToCart(jointAngles1, eeFrame);

	/***
	 * Below is an example on how to get the
	 * cartesian coordinates of EACH joint.
	***/

	eeFrame = Frame::Identity();
	eeFrame = eeFrame * kdlChain.getSegment(1).pose(jointAngles(0));
	print_end_effector(eeFrame);
	cout << endl;
	eeFrame = eeFrame * kdlChain.getSegment(2).pose(jointAngles(1));
	print_end_effector(eeFrame);
	cout << endl;
	eeFrame = eeFrame * kdlChain.getSegment(3).pose(0);
	print_end_effector(eeFrame);
	cout << endl;
	eeFrame = eeFrame * kdlChain.getSegment(4).pose(jointAngles(2));
	print_end_effector(eeFrame);

	// if (fk_status >= 0) {
	// 	print_end_effector(eeFrame);
	// } else {
	// 	printf("%s \n","Error: could not calculate forward kinematics");
	// }

	return 0;
}