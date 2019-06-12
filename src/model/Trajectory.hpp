#ifndef MODEL_TRAJECTORY_HPP
#define MODEL_TRAJECTORY_HPP

#include <vector>
#include "AngularKinematics.hpp"
#include "KinematicsModel.hpp"

class Trajectory {
	public:
		Trajectory();
		void init(int numJoints);
		void prepare(float start[NUM_OF_JOINTS], float end[NUM_OF_JOINTS], float duration);
		bool nextTimeStep(float tNow, float next[NUM_OF_JOINTS]);
		void finish();
		float timeNow();

	private:
		int _numJoints;
		typedef std::vector<AngularKinematics*> Joints;
		Joints _joints;
		float _tNow, _tEnd;
};

#endif