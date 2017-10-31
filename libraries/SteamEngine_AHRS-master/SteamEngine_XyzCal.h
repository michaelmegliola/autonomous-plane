#ifndef __STEAM_ENGINE_XYZCAL_H__
#define __STEAM_ENGINE_XYZCAL_H__

#include <SD.h>

enum XyzAxis { X, Y, Z };
enum XyzType { RAW, CALIBRATION, CORRECTED, FILTERED };

class XyzCal {
	public:
		XyzCal();
		void update(float* xyz);
		void accumulate(float* xyz);
		void calibrate(int divisor);
		void reset();
		float* getXyz(XyzType type);
		bool isMoving(float threshold);
		void dump(File* file);
				
	private:
		float vals[4][3];	// XyzType, XyzAxis
		float filter[4][3];	// buffer n, XyzAxis
		int lpf;
		float delta;
};

#endif
