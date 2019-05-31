#include <benchmark/benchmark.h>
#include "FwdKM.h"

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

BENCHMARK(BM_KM);
BENCHMARK_MAIN();