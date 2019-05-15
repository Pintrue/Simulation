#ifndef GUI_GLGRAPHICS_HPP
#define GUI_GLGRAPHICS_HPP

#include "Drawable.hpp"

class GLGraphics {
	public:
		GLGraphics();
		GLGraphics(const KinematicsModel& km);
		void render();

		Floor _floor;
		Model _model;
};

#endif