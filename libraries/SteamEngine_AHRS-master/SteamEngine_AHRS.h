#ifndef __STEAM_ENGINE_AHRS_H__
#define __STEAM_ENGINE_AHRS_H__

#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "SteamEngine_XyzCal.h"
#include "SteamEngine_LPF.h"

#define ACCEL_JITTER 0.50
#define MIN_CALIBRATION_COUNT 1000 
#define LOW_PASS_FILTER_SIZE 4
#define NULL 0

class SteamEngineAHRS
{
public:
  	float const PI_F = 3.14159265F;
	SteamEngineAHRS(Adafruit_Sensor* accelerometer,  Adafruit_Sensor* gyroscope,  Adafruit_BMP280* barometer, int ledPin);
	void recalibrate();
	void update();
	void describe(File *file) ;
	bool isCalibrated();
	bool isMoving();
	unsigned long getTimestamp();
	float getTimespan();
	float getTemperature();
	float* getGyro(XyzType type);
	float* getAccel(XyzType type);
	float getAltitude();
	float getPitch();
	float getRoll();

private:
	void calibrate();
	void resetCalibrationMetrics();
	void fillXyz(sensors_event_t* event, sensors_type_t type);

	//sensors and events
	Adafruit_Sensor* _accel;
	Adafruit_Sensor* _gyro;
	 Adafruit_BMP280* _bar;
 	sensors_event_t* accelEvent;
	sensors_event_t* gyroEvent;
	sensors_event_t* barEvent;
	
	//primary registers
	int led;
	unsigned long timestamp; 
	float timespan;
	float xyz[3];
	XyzCal* xyzGyro;
	XyzCal *xyzAccel;
	float altitude;
	float temperature;
	float pitch;
	float roll;
	float iGyroX;
	float iGyroY;
	float iGyroZ;
	
	float altitudeCal;
	int calibrationCount;
	boolean calibrated;

	//filters
	SteamEngineLPF* lpfAltitude;
};

#endif
