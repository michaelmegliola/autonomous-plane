#ifndef __STEAM_ENGINE_AHRS_H__
#define __STEAM_ENGINE_AHRS_H__

#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define ACCEL_JITTER 0.50
#define MIN_CALIBRATION_COUNT 1000 

class SteamEngineAHRS
{
public:
	SteamEngineAHRS(Adafruit_Sensor* accelerometer,  Adafruit_Sensor* gyroscope,  Adafruit_BMP280* barometer, int ledPin);
	void recalibrate();
	void update();
	void describe(File *file) ;
	boolean isCalibrated();
	unsigned long getTimestamp();
	float getTimespan();
	float getTemperature();
	float getAccX();
	float getAccY();
	float getAccZ();
	float getGyroX();
	float getGyroY();
	float getGyroZ();
	float getAltitude();

private:
	void calibrate();
	void resetCalibrationMetrics();

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
	float altitude;
	float temperature;
	float gyroX;
	float gyroY;
	float gyroZ;
	float accX;
	float accY;
	float accZ;

	//dynamically measured calibration constants
	float accXcal;
	float accYcal;
	float accZcal;
	float gyroXcal;
	float gyroYcal;
	float gyroZcal;
	float altitudeCal;
	int calibrationCount;
	boolean calibrated;
};

#endif
