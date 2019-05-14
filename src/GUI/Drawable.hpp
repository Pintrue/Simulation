#ifndef GUI_DRAWABLE_HPP
#define GUI_DRAWABLE_HPP

#include "../model/KinematicsModel.hpp"

#define POSE_DIM 6
#define RAD_TO_DEG(x) (180/M_PI)*x


// Abstract base class for any drawable objects
class Drawable {
	public:
		Drawable();
		virtual ~Drawable();
		virtual void draw() = 0; // pure virtual func.
		void setPose(double pose[POSE_DIM]);

		double _pose[POSE_DIM];
};


class Floor : public Drawable {
	public:
		Floor();
		~Floor();

		void draw() override;
	
	private:
		double startX, endX, /*startY, endY,*/ startZ, endZ;
};


class Jnt : public Drawable {
	public:
		Jnt();
		~Jnt();
		
		void draw() override;

	protected:
		double _height, _radius;
};


class BaseJoint : public Jnt {
	public:
		BaseJoint(double height, double radius);

		void draw() override;
};


class ArmJoint : public Jnt {
	public:
		ArmJoint(double height, double radius);

		void draw() override;
};


class ForearmJoint : public Jnt {
	public:
		ForearmJoint(double height, double radius);

		void draw() override;
};


class EndEffector : public Jnt {
	public:
		EndEffector(double radius);

		void draw() override;
};


class Model : public Drawable {
	public:
		Model();
		~Model();

		void init(const KinematicsModel& km);
		void finish();
		void update(const KinematicsModel& km, const KDL::JntArray& jnts);
		void draw() override;

	protected:

	    typedef std::vector<Drawable*> JointList;
	    JointList _joints;
};

#endif
