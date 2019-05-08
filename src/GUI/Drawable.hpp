#ifndef GUI_DRAWABLE_HPP
#define GUI_DRAWABLE_HPP

#include "../model/KinematicsModel.hpp"

#define POSE_DIM 6
#define RAD_TO_DEG(x) (180/M_PI)*x


// Abstract base class for any drawable objects
class Drawable {
	public:
		Drawable();
		virtual void draw() = 0; // pure virtual func.
		void setPose(double pose[POSE_DIM]);

	protected:
		double _pose[POSE_DIM];
};


class Floor : public Drawable {
	public:
		Floor();
		void draw() override;
	
	private:
		double startX, endX, startY, endY, startZ, endZ;
};


class Joint : public Drawable {
	public:
		Joint();
		
		void draw() override;

	protected:
		double _height, _radius;
};


class BaseJoint : public Joint {
	public:
		BaseJoint(double height, double radius);

		void draw() override;
};


class ArmJoint : public Drawable {
	public:
		ArmJoint(double height, double radius);

		void draw() override;
};


class ForearmJoint : public Drawable {
	public:
		ForearmJoint(double height, double radius);

		void draw() override;
};


class EndEffector : public Drawable {
	public:
		EndEffector(double radius);

		void draw() override;
};


class Model : public Drawable {
	public:
		Model();

		void init(const KinematicsModel& km);
		void finish();
		void update(const KinematicsModel& km, );
		void draw() override;
};

#endif
