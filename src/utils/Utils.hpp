#ifndef UTILS_UTILS_HPP
#define UTILS_UTILS_HPP

#include "../GUI/Drawable.hpp"


// void printFrame(const KDL::Frame& eeFrame);

void printPose(const double pose[6]);

// void convFrameToPose(const KDL::Frame& frame, double pose[6]);

// void printJntArray(const KDL::JntArray& ja);

bool withinCylinder(double center[CART_COORD_DIM], int radius,
					double obj[CART_COORD_DIM]);

bool illegalJntBoundary(const double* jntArray);

#endif
