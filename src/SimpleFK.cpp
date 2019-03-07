#include <iostream>
#include <iomanip>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

using namespace std;
using namespace KDL;

int main() {
    Chain kdlChain = Chain();

	// Construct segments: links of the arm
	// Joint joint1(Joint::None);
	// Frame frame1 = Frame(Vector(0.0, 1.0, 0.0));
	// kdlChain.addSegment(Segment(joint1, frame1));

	// Joint joint2(Joint::RotZ);
	// Frame frame2 = Frame(Vector(0.0, 2.0, 0.0));
	// kdlChain.addSegment(Segment(joint2, frame2));

	// Joint joint3(Joint::RotZ);
	// Frame frame3 = Frame(Rotation::EulerZYX(0.0, 0.0, -M_PI / 2)) * Frame(Vector(0.0, 0.0, 2.0));
	// kdlChain.addSegment(Segment(joint3, frame3));

	// Joint joint4(Joint::RotZ);
	// Frame frame4 = Frame(Rotation::EulerZYX(0.0, 0.0, M_PI / 2)) * Frame(Vector(1.0, 1.0, 0.0));
	// kdlChain.addSegment(Segment(joint4, frame4));

	// Joint joint1(Joint::None);
	// Frame frame1 = Frame(Rotation::EulerZYX(0.0, 0.0, -M_PI / 2)) * Frame(Vector(0.0, 0.0, 3.0));
	// kdlChain.addSegment(Segment(joint1, frame1));

	// Joint joint2(Joint::RotZ);
	// Frame frame2 = Frame(Rotation::EulerZYX(0.0, -M_PI / 2, 0)) * Frame(Vector(-4.5, -3.5, 0.0));
	// kdlChain.addSegment(Segment(joint2, frame2)); //3.5cm 7.5cm

	// Joint joint3(Joint::RotZ);
	// Frame frame3 = Frame(Vector(-1.0, 11.0, 0.0));
	// kdlChain.addSegment(Segment(joint3, frame3)); // 8.5cm

	// Joint joint4(Joint::RotZ);
	// Frame frame4 = Frame(Rotation::EulerZYX(0.0, M_PI / 2, 0.0)) * Frame(Rotation::EulerZYX(0.0, 0.0, M_PI / 2)) * Frame(Vector(0.0, 6.5, 14.0));
	// kdlChain.addSegment(Segment(joint4, frame4));

	kdlChain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0, 3.0, 0.0))));
	kdlChain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0, 4.5, 3.5))));
	kdlChain.addSegment(Segment(Joint(Joint::None),Frame(Vector(0.0, 1.0, -11.0))));
	kdlChain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0, 6.5, 14.0))));

	// Joint Angles
	// JntArray jointAngles = JntArray(3);
	// jointAngles(0) = -M_PI / 4.;       // Joint 1
	// jointAngles(1) = M_PI / 2.;        // Joint 2
	// jointAngles(2) = M_PI;             // Joint 3
	unsigned int nj = kdlChain.getNrOfJoints();
	cout << "Number of joints is " << nj << endl;
	if (nj != 3) {
		printf("%s \n","Error: Number of joints does not match.");
		return -1;
	}

	JntArray jointAngles = JntArray(3);
	jointAngles(0) = -M_PI / 2;			// Joint 1
	jointAngles(1) = 0;			// Joint 2
	jointAngles(2) = -1.135;			// Joint 3
	
	// Perform Forward Kinematics
	ChainFkSolverPos_recursive FKSolver = ChainFkSolverPos_recursive(kdlChain);
	Frame eeFrame;
	int fk_status;
	fk_status = FKSolver.JntToCart(jointAngles, eeFrame);

	if (fk_status >= 0) {
		// Print the frame
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
	} else {
		printf("%s \n","Error: could not calculate forward kinematics");
	}

	return 0;
}