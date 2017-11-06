#include "SteamEngine_XyzCal.h"

XyzCal::XyzCal(XyzSensor s, bool filtered) {
	sensor = s;
	hasFilter = filtered;
	reset();
}

void XyzCal::update(float* xyz, float seconds) {
	
	float* pidVec = hasFilter ? getXyz(FILTERED) : getXyz(CORRECTED);
	
	for (int i = 0; i < 3; i++) {
	//	trap prior value to allow calculation of rate of change (DERIVATIVE), below.
		float prior = pidVec[i];
	//	record new values in primary registers
		vals[RAW][i] = xyz[i];
	//	TODO: check for reading delta; ignore if out-of-bounds?
		vals[CORRECTED][i] = vals[RAW][i] - vals[CALIBRATION][i];
	//	apply low-pass filter (if enabled)
		if (hasFilter) {
			filter[lpf][i] = vals[CORRECTED][i];
			vals[FILTERED][i] = 0.0;
			for (int j = 0; j < 4; j++) {
				vals[FILTERED][i] += filter[j][i];
			}
			vals[FILTERED][i] /= 4.0;
		}
	//	calculate INTEGRAL (running sum) and DERIVATIVE (rate of change)	
		vals[INTEGRAL][i] += pidVec[i];	
		vals[DERIVATIVE][i] = (pidVec[i] - prior) / seconds;
	}
	lpf = ++lpf % 4;
}

void XyzCal::postCalibrate() {
	for (int i = 0; i < 3; i++) {
		vals[FILTERED][i] = 0.0;
		vals[INTEGRAL][i] = 0.0;
		vals[DERIVATIVE][i] = 0.0;
		for (int j = 0; j < 4; j++) {
			filter[j][i] = 0.0;
		}
	}
}

void XyzCal::accumulate(float* xyz) {
	for (int i = 0; i < 3; i++) {
		vals[CALIBRATION][i] += xyz[i];
	}
}

void XyzCal::calibrate(int divisor) {
	for (int i = 0; i < 3; i++) {
		vals[CALIBRATION][i] /= (float) divisor;
	}
	if (sensor == ACCEL) {
		vals[CALIBRATION][Z] -= SENSORS_GRAVITY_EARTH;
	}
}

void XyzCal::reset() {
//	registers
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 3; j++) {
			vals[i][j] = 0.0;
		}
	}
//	low-pass filter
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			filter[i][j] = 0.0;
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
		return hasFilter ? vals[FILTERED] : vals[CORRECTED];
	case INTEGRAL:
		return vals[INTEGRAL];
	case DERIVATIVE:
		return vals[DERIVATIVE];
	}
}	
		
void XyzCal::logHeader(File* file) {
	file->print("SENSOR,X-RAW,Y-RAW,Z-RAW,X-CAL,Y-CAL,Z-CAL,");
	file->print("X-P,Y-P,Z-P,X-PF,Y-PF,Z-PF,");
	file->print("X-I,Y-I,Z-I,X-D,Y-D,Z-D,");
}

void XyzCal::log(File* file) {
	file->print(sensor == ACCEL ? "ACCEL," : "GYRO,");
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 3; j++) {
			file->print(vals[i][j], 7);
			file->print(",");
		}
	}
}

