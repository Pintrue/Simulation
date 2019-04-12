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
    Chain kdlChain = Chain();

	// Construct segments: links of the arm
	kdlChain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0, 4.75, 0.0))));
	kdlChain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0, 4.47, 3.05))));
	kdlChain.addSegment(Segment(Joint(Joint::None),Frame(Vector(0.0, 1.43, -10.57))));
	kdlChain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0, 7.70, 16.2))));

	// Joint Angles
	unsigned int nj = kdlChain.getNrOfJoints();
	cout << "Number of joints is " << nj << endl;
	if (nj != 3) {
		printf("%s \n","Error: Number of joints does not match.");
		return -1;
	}

	JntArray jointAngles = JntArray(3);
	jointAngles(0) = 0;			// Joint 1
	jointAngles(1) = 0;			// Joint 2
	jointAngles(2) = -1.135;			// Joint 3
	
	// Perform Forward Kinematics
	ChainFkSolverPos_recursive FKSolver = ChainFkSolverPos_recursive(kdlChain);
	Frame eeFrame;
	int fk_status;
	fk_status = FKSolver.JntToCart(jointAngles, eeFrame);

	// Segment seg = kdlChain.getSegment(0);
	// Joint j = seg.getJoint();
	// Vector v = j.JointAxis();
	// cout << v << endl;

	Frame jtFrame = kdlChain.getSegment(3).getFrameToTip();
	print_end_effector(jtFrame);

	if (fk_status >= 0) {
		print_end_effector(eeFrame);
	} else {
		printf("%s \n","Error: could not calculate forward kinematics");
	}

	return 0;
}