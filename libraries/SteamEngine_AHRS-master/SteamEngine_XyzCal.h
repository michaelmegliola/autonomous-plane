/*
	Represents one set of calibrated readings from a sensor with three degrees of
	freedom (eg, an x-y-z accelerometer or x-y-z gyroscope). Includes an optional
	low-pass filter (to apply the filter, specify filter=TRUE when constructing 
	instance).

	Maintains most recent sensor values:
		RAW = most recent raw reading from sensor
		CALIBRATION = constant used to correct raw readings
		CORRECTED = RAW + CALIBRATION (corrected reading)
		FILTERED = corrected readings after application of optional low-pass filter (else zero)
		INTEGRAL = cumulative sum of FILTERED if availabe, otherwise CORRECTED
		DERIVATIVE = most recent change (per second) in FILTERED if available, otherwise CORRECTED
*/

#ifndef __STEAM_ENGINE_XYZCAL_H__
#define __STEAM_ENGINE_XYZCAL_H__

#include <Adafruit_Sensor.h>
#include <SD.h>

enum XyzSensor { ACCEL, GYRO };
enum XyzAxis { X, Y, Z };
enum XyzType { RAW, CALIBRATION, CORRECTED, FILTERED, INTEGRAL, DERIVATIVE };

class XyzCal {
	public:
		XyzCal(XyzSensor sensor, bool filtered);
		void update(float* xyz, float seconds);
		void accumulate(float* xyz);
		void calibrate(int divisor);
		void reset();
		float* getXyz(XyzType type);
		void logHeader(File* file);
		void log(File* file);
		void postCalibrate();
				
	private:
		bool hasFilter;
		float vals[6][3];	// XyzType, XyzAxis
		float filter[4][3];	// buffer n, XyzAxis
		int lpf;
		XyzSensor sensor;
};

#endif
