#ifndef __STEAM_ENGINE_XYZCAL_H__
#define __STEAM_ENGINE_XYZCAL_H__

#include <Adafruit_Sensor.h>
#include <SD.h>

enum XyzSensor { ACCEL, GYRO };
enum XyzAxis { X, Y, Z };
enum XyzType { RAW, CALIBRATION, CORRECTED, FILTERED, INTEGRAL, DERIVATIVE };

class XyzCal {
	public:
		XyzCal(XyzSensor sensor);
		void update(float* xyz);
		void accumulate(float* xyz);
		void calibrate(int divisor);
		void reset();
		float* getXyz(XyzType type);
		void dump(File* file);
				
	private:
		float vals[4][3];	// XyzType, XyzAxis
		float filter[4][3];	// buffer n, XyzAxis
		int lpf;
		XyzSensor sensor;
};

#endif
