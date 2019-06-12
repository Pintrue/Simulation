#ifndef UTILS_UTILS_HPP
#define UTILS_UTILS_HPP

#include "../GUI/Drawable.hpp"


// void printFrame(const KDL::Frame& eeFrame);

void printPose(const float pose[6]);

// void convFrameToPose(const KDL::Frame& frame, float pose[6]);

// void printJntArray(const KDL::JntArray& ja);

bool withinCylinder(float center[CART_COORD_DIM], int radius,
					float obj[CART_COORD_DIM]);

bool illegalJntBoundary(const float* jntArray);

#endif
