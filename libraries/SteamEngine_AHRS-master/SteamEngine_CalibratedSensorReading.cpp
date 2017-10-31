
CalibratedSensorReading::CalibratedSensorReading() {
	vals = new float[3][3];
}

void CalibratedSensorReading::update(float[] xyz) {
	for (int i = 0; i < 3; i++) {
		vals[RAW][i] = xyz[i];
		vals[COR][i] += vals[RAW][i] + vals[CAL][i];
	}
}

void CalibratedSensorReading::accumulate(float[] xyz) {
	for (int i = 0; i < 3; i++) {
		vals[RAW][i] += xyz[i];
	}
}

void CalibratedSensorReading::calibrate(int divisor) {
	for (int i = 0; i < 3; i++) {
		vals[CAL][i] += vals[RAW][i] / (float) divisor;
		vals[RAW][i] = 0.0;
	}
}

void CalibratedSensorReading::clear() {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			vals[i][j] = 0.0;
		}
	}
}

void float* getXyz() {
	return vals[COR];
}
				
