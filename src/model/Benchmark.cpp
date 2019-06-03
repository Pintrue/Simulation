#include <benchmark/benchmark.h>
#include "FwdKM.h"
#include <math.h>

static void BM_KM(benchmark::State& state) {
	double ja[3];
	double eePos[3];
	for (auto _ : state) {
		ja[0] = RAND_M_TO_N(JNT0_L, JNT0_U);
		ja[1] = RAND_M_TO_N(JNT1_L, JNT1_U);
		ja[2] = RAND_M_TO_N(JNT2_L, JNT2_U);
		initFwdKM();
		getEEPoseByJnts(ja, eePos);
	}
}


static void BM_KMPC(benchmark::State& state) {
	double ja[3];
	double eePos[3];
	initTrigTable();
	for (auto _ : state) {
		ja[0] = RAND_M_TO_N(JNT0_L, JNT0_U);
		ja[1] = RAND_M_TO_N(JNT1_L, JNT1_U);
		ja[2] = RAND_M_TO_N(JNT2_L, JNT2_U);
		initFwdKM();
		getEEPoseByJntsPrecomp(ja, eePos);
	}
}

static void BM_Sine(benchmark::State& state) {
	for (auto _ : state)
		sin(RAND_M_TO_N(-M_PI, M_PI));
}


static void BM_Cosine(benchmark::State& state) {
	for (auto _ : state)
		cos(RAND_M_TO_N(-M_PI, M_PI));
}


static void BM_SineTaylorApprox(benchmark::State& state) {
	for (auto _ : state)
		sineTaylorSeriesApprox(RAND_M_TO_N(-1, 1));
}


static void BM_SinePrecomp(benchmark::State& state) {
	initTrigTable();
	for (auto _ : state)
		sinePrecomp(RAND_M_TO_N(-M_PI, M_PI));
}


static void BM_CosinePrecomp(benchmark::State& state) {
	initTrigTable();
	for (auto _ : state)
		cosinePrecomp(RAND_M_TO_N(-M_PI, M_PI));
}


BENCHMARK(BM_Sine);
BENCHMARK(BM_SinePrecomp);
// BENCHMARK(BM_Cosine);
// BENCHMARK(BM_CosinePrecomp);

// BENCHMARK(BM_KM);
// BENCHMARK(BM_KMPC);
BENCHMARK(BM_SineTaylorApprox);

BENCHMARK_MAIN();