#ifndef MODEL_ANGULAR_KINEMATICS_HPP
#define MODEL_ANGULAR_KINEMATICS_HPP

class AngularKinematics {
	public:
		struct TemporalData {
			double angle;
			double velocity;
		};
		void init(TemporalData start, TemporalData end, double t);
		void angleAtTime(double t, double* angle);

	private:
		struct Terms {
			double c0;
			double c1;
			double c2;
			double c3;
		};
		struct Terms _cache; 
};
	
#endif