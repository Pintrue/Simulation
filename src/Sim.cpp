#include "Sim.hpp"
#include "utils/Utils.hpp"

using namespace std;
using namespace KDL;

int main() {
    Sim sim;

    // origin where the robot is based
    double pos[3] = {0.0, 0.0, 0.0};

    // initialize kinematics and trajectory models
    sim._km.init(pos);
    sim._tjt.init(3);

    JntArray fromJA = JntArray(3);
    SetToZero(fromJA);

    // JntArray toJA = JntArray(3);
	// toJA(0) = M_PI / 2;			// Joint 1
	// toJA(1) = 0;			        // Joint 2
	// toJA(2) = -(80.0/180.0*M_PI);// Joint 3
    JntArray toJA = JntArray(3);
	toJA(0) = M_PI;			// Joint 1
	toJA(1) = 0;			        // Joint 2
	toJA(2) = 0;// Joint 3

    JntArray interJA = JntArray(3);

    sim._tjt.prepare(fromJA, toJA, 10);
    
    for (int i = 1; i <= 10; ++i) {
        if (sim._tjt.nextTimeStep(sim._tjt.timeNow() + 1, interJA))
            cout << "Ongoing!" << endl;
    }

    Frame eeFrame;
    sim._km.jntsToCart(toJA, eeFrame);
    print_frame(eeFrame);
    cout << "Done" << endl;
}