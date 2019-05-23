#ifndef GUI_GLGRAPHICS_HPP
#define GUI_GLGRAPHICS_HPP

#include "Drawable.hpp"
#include "../Sim.hpp"

class GLGraphics {
	public:
		GLGraphics();
		GLGraphics(const KinematicsModel& km);
		void render();

		Floor _floor;
		Model _model;
		Goal _goal;
		Obj _obj;
};

#endif