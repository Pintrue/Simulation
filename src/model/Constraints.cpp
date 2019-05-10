#include "Constraints.hpp"
#include <math.h>

using namespace std;

void regulateJntAngles(const KDL::JntArray& jntAngles,
						const double delta[ACTION_DIM],
						double result[FULL_STATE_NUM_COLS]) {
	result[0] = min(max(jntAngles(0)+delta[0], JA0_L), JA0_U);
	result[1] = min(max(jntAngles(1)+delta[1], JA1_L), JA1_U);
	result[2] = min(max(jntAngles(2)+delta[2], JA2_L), JA2_U);
}


bool inGroundLvlWorkspace(double pos[CART_DIM]) {
	if (pos[1] != 0) {
		return false;
	}
	double dist = sqrt(pow(pos[0], 2) + pow(pos[2], 2));

	return (dist > GROUND_INN_RADIUS) && (dist < GROUND_OUT_RADIUS);
}


// int main() {
// 	double pos1[3] = {-16.5, 1, 0};
// 	cout << inGroundLvlWorkspace(pos1) << endl;

// 	while (1) {
// 		double dest[CART_DIM];
// 		do {
// 			dest[0] = rand_uniform(-GROUND_OUT_RADIUS, GROUND_OUT_RADIUS);
// 			dest[1] = 0;
// 			dest[2] = rand_uniform(-GROUND_OUT_RADIUS, GROUND_OUT_RADIUS);
// 		} while (!inGroundLvlWorkspace(dest));

// 		for (int i = 0; i < 3; ++i) {
// 			cout << dest[i] << " ";
// 		}

// 		cout << endl;
// 	}
// }