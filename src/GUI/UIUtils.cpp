#include "UIUtils.hpp"
#include <QtCore/QRegExp>
#include <QtCore/QStringList>
#include "../model/Constraints.hpp"

#include <iostream>
using namespace std;


bool isValidJAFormat(const QString& angleInput, double result[NUM_OF_JOINTS]) {
	/**
	 * Ignore empty entries after split
	 * e.g. x = "a,,b,c";
	 * 		x.split("[, ]");					// return ["a","","b","c"]
	 * 		x.split("[, ]", SkipEmptyParts);	// return ["a","b","c"]
	**/
	QRegExp regex("[, ]");
	QStringList angles = angleInput.split(regex, QString::SkipEmptyParts);

	if (angles.length() != NUM_OF_JOINTS) {
		return false;
	}

	double jntAngles[NUM_OF_JOINTS];
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		bool digit;
		jntAngles[i] = angles[i].toDouble(&digit);
		if (!digit)
			return false;
	}

	return areInValidRanges(jntAngles, result);
}


// int main() {
// 	QString str;
// 	str = "1.5, 0, 0";
// 	double res[3];
// 	cout <<isValidJAFormat(str, res) << endl;
// 	for (int i = 0; i < 3; ++i)
// 		cout << res[i] << " ";
// 	cout << endl;
// }