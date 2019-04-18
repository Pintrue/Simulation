#include "Sim.hpp"
#include "utils/Utils.hpp"

using namespace std;
using namespace KDL;

int main() {
    KinematicsModel km;

    double pos[3] = {0.0, 0.0, 0.0};
    km.init(pos);

    JntArray jointAngles = JntArray(3);
	jointAngles(0) = M_PI / 2;			// Joint 1
	jointAngles(1) = 0;			// Joint 2
	jointAngles(2) = -(80.0/180.0*M_PI);			// Joint 3

    Frame eeFrame;
    km.jntsToCart(jointAngles, eeFrame);
    print_frame(eeFrame);
    cout << "Done" << endl;
}