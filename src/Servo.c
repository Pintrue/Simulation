#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <pigpio.h>

#include "model.h"
#include "model_utils.h"
#include "matrix_op.h"
#include "FwdKM.h"
#include <math.h>

#define NUM_OF_SERVO 3
#define BASE_SERVO_GPIO_PIN 4
#define SHOULDER_SERVO_GPIO_PIN 17
#define FOREARM_SERVO_GPIO_PIN 22
#define WRIST_SERVO_GPIO_PIN 13
#define MAGNET_GPIO_PIN 26

#define BASE_RANGE M_PI
#define SHOULDER_RANGE ((115.0 / 180.0) * M_PI)
//#define FOREARM_RANGE (M_PI / 2.0)
#define FOREARM_RANGE ((88.0 / 180.0) * M_PI)
//#define WRIST_RANGE (M_PI / 2 + 12.0/180.0*M_PI)
#define WRIST_RANGE M_PI

#define BASE_MID 76
//#define BASE_RIGHT 116
#define BASE_RIGHT 125
//#define BASE_LEFT 32
#define BASE_LEFT 23
#define SHOULDER_LEFT 83
#define SHOULDER_RIGHT 26
#define FOREARM_RIGHT 76
//#define FOREARM_RIGHT 65
#define FOREARM_LEFT 32
#define WRIST_DOWN 35
#define WRIST_UP 118
#define WRIST_LIMIT 70

#define OBJ_HEIGHT 0.5
#define OBJ_ATTACHED_RANGE 1.0

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

int run = 1;

int used[3] = {BASE_SERVO_GPIO_PIN, SHOULDER_SERVO_GPIO_PIN, FOREARM_SERVO_GPIO_PIN};
double jntAngles[3];
double goal[3];
double object[3];
int hasObj;


void init() {
	if (gpioInitialise() < 0) {
		exit(0);
	}

	/* Initialize the three servo signals */
	for (int i = 0; i < NUM_OF_SERVO; ++i) {
		int pin = used[i];
		gpioSetPWMfrequency(pin, 50);
		gpioSetPWMrange(pin, 1000);

		printf("Range for %d is %d\n", pin, gpioGetPWMrange(pin));
		printf("Frequency for %d is %d\n", pin, gpioGetPWMfrequency(pin));
	}

	for (int i = 0; i < 3; ++i) {
		jntAngles[i] = 0;
	}

//	jntAngles[2] = -(17.0 / 180.0) * M_PI;

	gpioSetPWMfrequency(WRIST_SERVO_GPIO_PIN, 50);
	gpioSetPWMrange(WRIST_SERVO_GPIO_PIN, 1000);
	printf("Range for %d is %d\n", WRIST_SERVO_GPIO_PIN, gpioGetPWMrange(WRIST_SERVO_GPIO_PIN));
	printf("Frequency for %d is %d\n", WRIST_SERVO_GPIO_PIN, gpioGetPWMfrequency(WRIST_SERVO_GPIO_PIN));
	gpioPWM(WRIST_SERVO_GPIO_PIN, WRIST_LIMIT);

	gpioSetMode(MAGNET_GPIO_PIN, PI_OUTPUT);
	// gpioPWM(MAGNET_GPIO_PIN, gpioGetPWMrange(MAGNET_GPIO_PIN) / 2);
	// gpioPWM(MAGNET_GPIO_PIN, 140);
	printf("Range for %d is %d\n", MAGNET_GPIO_PIN, gpioGetPWMrange(MAGNET_GPIO_PIN));

	hasObj = 0;
}


matrix_t* initObsReaching(double target[3]) {
	matrix_t* obs = new_matrix(1, 9);
	double* data = obs->data;

	for (int i = 0; i < 3; ++i) {
		data[i] = jntAngles[i];
	}

	double eePos[6];
	initFwdKM();
	int res = getMagnetPoseByJnts(jntAngles, eePos);

	if (res < 0) {
		printf("1. Joint angle moved out of bounds.\n");
		exit(0);
	}

	for (int i = 0; i < 3; ++i) {
		data[i + 3] = eePos[i];
	}

	for (int i = 0; i < 3; ++i) {
		data[i + 6] = target[i];
		goal[i] = target[i];
	}

	return obs;
}


matrix_t* initObsPnP(double target[3], double obj[3]) {
	matrix_t* obs = new_matrix(1, 14);
	double* data = obs->data;

	for (int i = 0; i < 3; ++i) {
		data[i] = jntAngles[i];
	}

	double eePos[6];
	initFwdKM();
	int res = getMagnetPoseByJnts(jntAngles, eePos);

	if (res < 0) {
		printf("1. Joint angle moved out of bounds.\n");
		exit(0);
	}

	for (int i = 0; i < 3; ++i) {
		data[i + 3] = eePos[i];
	}

	data[6] = 0;
	
	for (int i = 0; i < 3; ++i) {
		data[i + 7] = obj[i];
		object[i] = obj[i];
	}

	data[10] = 0;

	for (int i = 0; i < 3; ++i) {
		data[i + 11] = target[i];
		goal[i] = target[i];
	}

	return obs;
}


void convertAnglesToDC(int dutyCycles[3], double shoulderDelta) {
	/* BASE SERVO */
	double m0 = ((double)(BASE_RIGHT - BASE_LEFT)) / BASE_RANGE;
	double b0 = BASE_MID;
	printf("m0 is %f and b0 is %f", m0, b0);
	dutyCycles[0] = (int) roundf(m0 * jntAngles[0] + b0);
	printf("Base DC is %d\n", dutyCycles[0]);

	/* SHOULDER SERVO */
	double m1 = ((double)(SHOULDER_RIGHT - SHOULDER_LEFT)) / SHOULDER_RANGE;
	double b1 = SHOULDER_LEFT;
 
	dutyCycles[1] = (int) roundf(m1 * jntAngles[1] + b1);
	printf("Shoulder DC is %d\n", dutyCycles[1]);

	/* FOREARM SERVO */
	double m2 = ((double)(FOREARM_RIGHT - FOREARM_LEFT)) / FOREARM_RANGE;
	double b2 = FOREARM_RIGHT;

	dutyCycles[2] = (int) roundf(m2 * jntAngles[2] + b2);
	dutyCycles[2] += (SHOULDER_LEFT - dutyCycles[1]) + 5;
	printf("Forearm DC is %d\n", dutyCycles[2]);
}


matrix_t* denormalize_action(matrix_t* action) {
	matrix_t* ret = new_matrix(action->rows, action->cols);
	double upper = 0.0872664626;
	double lower = -0.0872664626;
	double d1 = (double)(action->data[0] + 1) / (double)2 * (upper - lower) + lower;
	double d2 = (double)(action->data[1] + 1) / (double)2 * (upper - lower) + lower;
	double d3 = (double)(action->data[2] + 1) / (double)2 * (upper - lower) + lower;
	ret->data[0] = d1;
	ret->data[1] = d2;
	ret->data[2] = d3;
	return ret;
}


matrix_t* evalStepReaching(matrix_t* action) {
	matrix_t* newObs = new_matrix(1, 9);
	double* data = newObs->data;

	matrix_t* denormedAction = denormalize_action(action);
	print_matrix(denormedAction, 1);
	for (int i = 0; i < 3; ++i) {
		jntAngles[i] += denormedAction->data[i];
	}

	for (int i = 0; i < 3; ++i) {
		data[i] = jntAngles[i];
	}

	double eePos[6];
	initFwdKM();

	int res = getMagnetPoseByJnts(jntAngles, eePos);

	if (res < 0) {
		print_matrix(action, 1);
		printf("2. Joint angle moved out of bound.\n");
		exit(0);
	}

	int dutyCycles[3];
	convertAnglesToDC(dutyCycles, denormedAction->data[1]);

	for (int i = 0; i < 3; ++i) {
		gpioPWM(used[i], dutyCycles[i]);
		time_sleep(0.5);
	}

	double angle = getCache()->a4;
	printf("The wrist angle is %f\n", angle);

	double mWrist = - (WRIST_UP - WRIST_DOWN) / WRIST_RANGE;
	double bWrist = WRIST_UP;

	angle = MAX(MIN(angle + M_PI/2, WRIST_RANGE), 0);
	double wristDC = (int) roundf(mWrist * angle + bWrist);
	if (wristDC < WRIST_LIMIT) {
		wristDC = WRIST_LIMIT;
	}
	gpioPWM(WRIST_SERVO_GPIO_PIN, wristDC);
	time_sleep(0.5);

	for (int i = 0; i < 3; ++i) {
		data[i + 3] = eePos[i];
	}

	for (int i = 0; i < 3; ++i) {
		data[i + 6] = goal[i];
	}

	return newObs;
}


int ifHadObjPnP(const double fullState[14]) {
	double diff = 0;

	for (int i = 0; i < 3; ++i) {
		double delta = fullState[i + 3] - object[i];
		diff += delta * delta;
	}

	return sqrt(diff) <= OBJ_ATTACHED_RANGE;
}


matrix_t* evalStepPnP(matrix_t* action) {
	matrix_t* newObs = new_matrix(1, 14);
	double* data = newObs->data;

	matrix_t* jaAction = new_matrix(1, 3);
	jaAction->data[0] = action->data[0];
	jaAction->data[1] = action->data[1];
	jaAction->data[2] = action->data[2];

	matrix_t* denormedJA = denormalize_action(jaAction);
	matrix_t* denormedAction = new_matrix(1, 4);
	for (int i = 0; i < 3; ++i) {
		denormedAction->data[i] = denormedJA->data[i];
	}
	free_matrix(jaAction);
	free_matrix(denormedJA);
	denormedAction->data[3] = action->data[3];

	print_matrix(denormedAction, 1);
	for (int i = 0; i < 3; ++i) {
		jntAngles[i] += denormedAction->data[i];
	}

	for (int i = 0; i < 3; ++i) {
		data[i] = jntAngles[i];
	}

	double eePos[6];
	initFwdKM();

	int res = getMagnetPoseByJnts(jntAngles, eePos);

	if (res < 0) {
		print_matrix(action, 1);
		printf("2. Joint angle moved out of bound.\n");
		exit(0);
	}

	int dutyCycles[3];
	convertAnglesToDC(dutyCycles, denormedAction->data[1]);

	for (int i = 0; i < 3; ++i) {
		gpioPWM(used[i], dutyCycles[i]);
		time_sleep(0.5);
	}

	double angle = getCache()->a4;
	printf("The wrist angle is %f\n", angle);

	double mWrist = - (WRIST_UP - WRIST_DOWN) / WRIST_RANGE;
	double bWrist = WRIST_UP;

	angle = MAX(MIN(angle + M_PI/2, WRIST_RANGE), 0);
	double wristDC = (int) roundf(mWrist * angle + bWrist);
	if (wristDC < WRIST_LIMIT) {
		wristDC = WRIST_LIMIT;
	}
	gpioPWM(WRIST_SERVO_GPIO_PIN, wristDC);
	time_sleep(0.5);

	for (int i = 0; i < 3; ++i) {
		data[i + 3] = eePos[i];
	}

	data[6] = round(denormedAction->data[3]);

	if (data[6] == 1) {
		gpioPWM(MAGNET_GPIO_PIN, 140);
	} else {
		gpioPWM(MAGNET_GPIO_PIN, 0);
	}

	if (hasObj == 1 && data[6] == 1) {
		data[10] = 1;

		for (int i = 0; i < 3; ++i) {
			data[i + 7] = eePos[i];
			object[i] = eePos[i];
		}
	} else if (hasObj == 1 && data[6] == 0) {
		data[10] = 0;

		object[1] = OBJ_HEIGHT;
		hasObj = 0;
		for (int i = 0; i < 3; ++i) {
			data[i + 7] = object[i];
		}
	} else if (hasObj == 0 && data[6] == 1) {
		if (ifHadObjPnP(data)) {
			hasObj = 1;
			data[10] = 1;

			for (int i = 0; i < 3; ++i) {
				data[i + 7] = eePos[i];
				object[i] = eePos[i];
			}
		} else {
			data[10] = 0;
		
			for (int i = 0; i < 3; ++i) {
				data[i + 7] = object[i];
			}
		}
	} else {
		data[10] = 0;
		
		for (int i = 0; i < 3; ++i) {
			data[i + 7] = object[i];
		}
	}

	for (int i = 0; i < 3; ++i) {
		data[i + 11] = goal[i];
	}

	return newObs;
}


void stop(int signum) {
	run = 0;
	finishFwdKM();
	gpioPWM(MAGNET_GPIO_PIN, 0);
	time_sleep(1);
	gpioTerminate();
	printf("Exit gracefully...\n");
	exit(0);
}


int main(int argc, char *argv[]) {
	init();
	signal(SIGINT, stop);
	gpioSetSignalFunc(SIGINT, stop);

/*
        jntAngles[0] = -0.2;
        jntAngles[1] = 0.7;
        jntAngles[2] = -0.3;

        int dc[3];
        convertAnglesToDC(dc, jntAngles[1]);
	dc[0] = 66; dc[1] = 38; dc[2] = 98;
        for (int i = 0; i < 3; ++i) {
                gpioPWM(used[i], dc[i]);
                time_sleep(0.5);
        }

        int n;
        scanf("%d", &n);
*/
/*
        while (1) {
                int stop = 0;
                scanf("%d", &stop);
                if (stop == 1) {
                        // gpioWrite(MAGNET_GPIO_PIN, 0);
                        gpioPWM(MAGNET_GPIO_PIN, 0);
                        return 0;
                }
 is 33                time_sleep(0.5);
        }
*/

/*
	// Reaching main
	double target[3] = { -7.495477e+00,0.000000e+00,1.870172e+01 };
	matrix_t* obs = initObsReaching(target);
	print_matrix(obs, 1);

	model* model = load_model("DDPG_ACTOR_SIM_NORM.model");
	normalizer* norm = load_normalizer("DDPG_NORM_SIM_NORM.norm");

	for (int i = 0; i < 50; ++i) {
		print_matrix(obs, 1);
		normalize_obs(norm, obs);
		predict(model, obs);
		obs = evalStepReaching(obs);
		printf("Done %d\n", i);
		time_sleep(0.5);
	}

	while (1) {
        int stop = 0;
        scanf("%d", &stop);
        if (stop == 1) {
            gpioPWM(MAGNET_GPIO_PIN, 0);
            return 0;
        }
        time_sleep(0.5);
    }
*/

	// PnP main
//	double obj[3] = { -7.495477e+00,0.000000e+00,1.870172e+01 };
	double target[3] = { -7.495477e+00,0.000000e+00,1.870172e+01 };

	double obj[3] = {5.0, 0.0, 20.0};
//	double target[3] = {5.0, 0.0, 20.0};

	matrix_t* obs = initObsPnP(target, obj);
	print_matrix(obs, 1);

	model* model = load_model("DDPG_ACTOR_PICKNPLACE_NORM_SIM.model");
	normalizer* norm = load_normalizer("DDPG_NORM_PICKNPLACE_NORM_SIM.norm");

	for (int i = 0; i < 50; ++i) {
		print_matrix(obs, 1);
		normalize_obs(norm, obs);
		predict(model, obs);
		obs = evalStepPnP(obs);
		printf("Done %d\n", i);
		time_sleep(0.5);
	}


/*
	 for (int i = 0; i < 10; ++i) {
		int base;
//	 	int shoulder;
//	 	int fore_arm;
	//	int wrist;
		scanf("%d", &base);

//	 	scanf("%d", &shoulder);
//		scanf("%d", &fore_arm);

	// 	//scanf("fore %d\n", &fore_arm);
	 	gpioPWM(BASE_SERVO_GPIO_PIN, base);
//	 	gpioPWM(SHOULDER_SERVO_GPIO_PIN, shoulder);
//		gpioPWM(FOREARM_SERVO_GPIO_PIN, fore_arm);
	 	time_sleep(0.5);
	 }
*/
	finishFwdKM();
	gpioPWM(MAGNET_GPIO_PIN, 0);
	time_sleep(1);
	gpioTerminate();

	return 0;
}
