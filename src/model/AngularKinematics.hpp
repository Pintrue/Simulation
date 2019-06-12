#ifndef MODEL_ANGULAR_KINEMATICS_HPP
#define MODEL_ANGULAR_KINEMATICS_HPP

class AngularKinematics {
	public:
		struct TemporalData {
			float angle;
			float velocity;
		};
		void init(TemporalData start, TemporalData end, float t);
		void angleAtTime(float t, float* angle);

	private:
		struct Terms {
			float c0;
			float c1;
			float c2;
			float c3;
		};
		struct Terms _cache; 
};
	
#endif