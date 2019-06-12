#include "Constraints.hpp"
#include <math.h>

using namespace std;


bool areInValidRanges(float jntAngles[NUM_OF_JOINTS],
						float result[NUM_OF_JOINTS]) {
	result[0] = (float)min(max((double)jntAngles[0], JA0_L), JA0_U);
	result[1] = (float)min(max((double)jntAngles[1], JA1_L), JA1_U);
	result[2] = (float)min(max((double)jntAngles[2], JA2_L), JA2_U);
	return (jntAngles[0] == result[0]) && (jntAngles[1] == result[1]) && (jntAngles[2] == result[2]);
}


// whether get regulated - false -> invalid; true -> valid
bool regulateJntAngles(float jntAngles[NUM_OF_JOINTS],
						const float delta[ACTION_DIM],
						float result[FULL_STATE_NUM_COLS]) {
	float ja[NUM_OF_JOINTS];
	ja[0] = jntAngles[0] + delta[0];
	ja[1] = jntAngles[1] + delta[1];
	ja[2] = jntAngles[2] + delta[2];

	float check[NUM_OF_JOINTS];
	if (areInValidRanges(ja, check)) {
		for (int i = 0; i < NUM_OF_JOINTS; ++i) {
			result[i] = check[i];
		}
		return true;
	} else {
		return false;
	}
	// result[0] = min(max(ja0, JA0_L), JA0_U);
	// result[1] = min(max(ja1, JA1_L), JA1_U);
	// result[2] = min(max(ja2, JA2_L), JA2_U);
	
}


bool inGroundLvlWorkspace(float pos[CART_DIM]) {
	if (pos[1] != 0) {
		return false;
	}
	float dist = sqrt(pow(pos[0], 2) + pow(pos[2], 2));

	return (dist > GROUND_INN_RADIUS) && (dist < GROUND_OUT_RADIUS);
}


// int main() {
// 	float pos1[3] = {-16.5, 1, 0};
// 	cout << inGroundLvlWorkspace(pos1) << endl;

// 	while (1) {
// 		float dest[CART_DIM];
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