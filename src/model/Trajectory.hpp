#ifndef MODEL_TRAJECTORY_HPP
#define MODEL_TRAJECTORY_HPP

#include <vector>
#include "AngularKinematics.hpp"
#include "KinematicsModel.hpp"

class Trajectory {
	public:
		Trajectory();
		void init(int numJoints);
		void prepare(double start[NUM_OF_JOINTS], double end[NUM_OF_JOINTS], double duration);
		bool nextTimeStep(double tNow, double next[NUM_OF_JOINTS]);
		void finish();
		double timeNow();

	private:
		int _numJoints;
		typedef std::vector<AngularKinematics*> Joints;
		Joints _joints;
		double _tNow, _tEnd;
};

#endif