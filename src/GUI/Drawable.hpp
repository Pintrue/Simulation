#ifndef GUI_DRAWABLE_HPP
#define GUI_DRAWABLE_HPP

#define POSE_DIM 6


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
		void draw();
	
	private:
		double startX, endX, startY, endY, startZ, endZ;
};


class Model : public Drawable {
	public:
		void draw();
};

#endif
