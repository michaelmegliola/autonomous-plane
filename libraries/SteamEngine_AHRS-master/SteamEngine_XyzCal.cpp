#include "SteamEngine_XyzCal.h"

XyzCal::XyzCal() {
	reset();
}

void XyzCal::update(float* xyz) {
	float maxDelta = abs(xyz[X] - vals[RAW][X]);
	for (int i = 0; i < 3; i++) {
		float d = abs(xyz[i] - vals[RAW][i]);
		maxDelta = ( d > maxDelta ) ? d : maxDelta;
		vals[RAW][i] = xyz[i];
		//TODO: check for reading delta; ignore if out-of-bounds
		vals[CORRECTED][i] = vals[RAW][i] + vals[CALIBRATION][i];
		filter[lpf][i] = vals[CORRECTED][i];
	}
	lpf = ++lpf % 4;
	delta = maxDelta;
}

void XyzCal::accumulate(float* xyz) {
	for (int i = 0; i < 3; i++) {
		vals[RAW][i] += xyz[i];
	}
}

void XyzCal::calibrate(int divisor) {
	for (int i = 0; i < 3; i++) {
		vals[CALIBRATION][i] += vals[RAW][i] / (float) divisor;
		vals[RAW][i] = 0.0;
	}
}

void XyzCal::reset() {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			vals[i][j] = 0.0;
		}
	}
	lpf = 0;
}

float* XyzCal::getXyz(XyzType type) {
	switch (type) {
	case RAW:
		return vals[RAW];
	case CALIBRATION:
		return vals[CALIBRATION];
	case CORRECTED:
		return vals[CORRECTED];
	case FILTERED:
		for (int i = 0; i < 3; i++) {
			vals[FILTERED][i] = 0.0;
		}
		for (int i=0; i < 4; i++) {
			for (int j = 0; j < 3; j++) {
				vals[FILTERED][j] += filter[i][j];
			}
		}
		for (int i = 0; i < 3; i++) {
			vals[FILTERED][i] /= 4.0;
		}
		return vals[FILTERED];	
	}
}

bool XyzCal::isMoving(float threshold) {
	return delta > threshold;
}

void XyzCal::dump(File* file) {
	file->println("START OF REGISTERS");
	file->println("RAW,CALIBRATION,CORRECTED,FILTERED");
	getXyz(FILTERED); //refresh filter
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			file->print(vals[i][j], 12);
		}
		file->println();
	}
	file->println("END OF REGISTERS");
	file->println("START OF LPF BUFFERS");
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			file->print(filter[i][j], 12);
		}
		file->println();
	}
	file->println("END OF LPF BUFFERS");
	file->flush();
}

