#ifndef MODEL_TRAJECTORY_HPP
#define MODEL_TRAJECTORY_HPP

#include <vector>
#include "AngularKinematics.hpp"
#include "kdl/jntarray.hpp"

class Trajectory {
	public:
		Trajectory();
		void init(int numJoints);
		void prepare(KDL::JntArray& start, KDL::JntArray& end, double duration);
		bool nextTimeStep(double tNow, KDL::JntArray& next);
		void finish();
		double timeNow();

	private:
		int _numJoints;
		typedef std::vector<AngularKinematics*> Joints;
		Joints _joints;
		double _tNow, _tEnd;
};

#endif