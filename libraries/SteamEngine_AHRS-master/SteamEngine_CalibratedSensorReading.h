#ifndef __STEAM_ENGINE_CSR_H__
#define __STEAM_ENGINE_CSR_H__

#define X 0
#define Y 1
#define Z 2
#define RAW 0 //raw value
#define CAL 1 //calibration constant
#define COR 2 //corrected value

class CalibratedSensorReading {

	public:
		CalibratedSensorReading();
		void update(float[] rawVals);
		void accumulate(float[] rawVals);
		void calibrate(int divisor);
		void clear();
		float* getXyz();
				
	private:
		float[][] vals;
	
};

#endif