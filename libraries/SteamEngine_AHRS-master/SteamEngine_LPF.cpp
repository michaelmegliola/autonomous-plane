#include "SteamEngine_LPF.h"

SteamEngineLPF::SteamEngineLPF(int buffersize) {
	size = buffersize;
	buf = new float[size];
	for (int n = 0; n < size; n++) buf[n] = 0.0;
	i = 0;
}

void SteamEngineLPF::update(float val) {
	buf[i] = val;
	i = ++i % size;
}

float SteamEngineLPF::getFilteredVal() {
	float sum = 0.0;
	for (int n = 0; n < size; n++) sum += buf[n];
	return sum / (float) size;
}
