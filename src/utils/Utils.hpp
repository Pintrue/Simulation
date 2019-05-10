#ifndef UTILS_UTILS_HPP
#define UTILS_UTILS_HPP

#include "../GUI/Drawable.hpp"
#include <kdl/frames_io.hpp>

void printFrame(const KDL::Frame& eeFrame);

void convFrameToPose(const KDL::Frame& frame, double pose[POSE_DIM]);

#endif
