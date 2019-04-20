#include "AngularKinematics.hpp"


void AngularKinematics::init(AngularKinematics::TemporalData start,
							AngularKinematics::TemporalData end, double t) {

	double sA, fA, sV, fV;
	sA = start.angle;
	fA = end.angle;
	sV = start.velocity;
	fV = end.velocity;

	double dA = fA - sA;
	_cache.c0 = sA;
	_cache.c1 = sV;
	_cache.c2 = (3.0*dA-(2.0*sV+fV)*t)/(t*t);
	_cache.c3 = (-2.0*dA+(sV+fV)*t)/(t*t*t);
}

void AngularKinematics::angleAtTime(double t, double* angle) {
	// theta = theta_i + omega_i*t + 1/2*alpha*t^2
	*angle = _cache.c0 + t * (_cache.c1 + t * (_cache.c2 + (t * _cache.c3)));
}