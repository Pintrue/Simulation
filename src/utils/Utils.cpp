//
// Created by Pinchu on 2019/3/7.
//
#include <stdio.h>
#include "Utils.hpp"

using namespace std;

void print_frame(const KDL::Frame &eeFrame) {// Print the frame
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