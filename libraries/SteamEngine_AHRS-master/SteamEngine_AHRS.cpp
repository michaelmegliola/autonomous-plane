/*  =============================================================================
 *  Sensor Readings
 *  
 *  Represents a single, complete set of sensor readings at a 
 *  specified point in time (to the nearest millisecond).
 */

#include "SteamEngine_AHRS.h"
 
	SteamEngineAHRS::SteamEngineAHRS(Adafruit_Sensor* accelerometer,  Adafruit_Sensor* gyroscope,   Adafruit_BMP280* barometer, int ledPin) {
		_accel = accelerometer;
		_gyro = gyroscope;
		_bar = barometer;
		led = ledPin;
		accelEvent = new sensors_event_t();
		gyroEvent = new sensors_event_t();
		barEvent = new sensors_event_t();
		resetCalibrationMetrics();
	}

	void SteamEngineAHRS::update() { 
		_gyro->getEvent(gyroEvent);
		_accel->getEvent(accelEvent);
		timespan = (float) (gyroEvent->timestamp - timestamp) / 1000.0;
		timestamp = gyroEvent->timestamp;
		altitude = _bar->readAltitude(1013.25);
		temperature = _bar->readTemperature();
		gyroX = gyroEvent->gyro.x;
		gyroY = gyroEvent->gyro.y;
		gyroZ = gyroEvent->gyro.z;
		accX = accelEvent->acceleration.x;
		accY = accelEvent->acceleration.y;
		accZ = accelEvent->acceleration.z;
	}

	void SteamEngineAHRS::recalibrate() {
		unsigned long start = millis();
		boolean flash = false;
		while (!calibrated) {
			calibrate();
			if (millis() > start + 50) {   
				digitalWrite(led, flash ? HIGH : LOW);
				flash = !flash; 
				start = millis();
			}
		}
		digitalWrite(RED_LED, LOW);
	}

 	void SteamEngineAHRS::describe(File *file) {
		file->println("START SENSOR CALIBRATION");
		file->print("accXcal,");
		file->println(accXcal, 12);
		file->print("accYcal,");
		file->println(accYcal, 12);
		file->print("accZcal,");
		file->println(accZcal, 12);
		file->print("gyroXcal,");
		file->println(gyroXcal, 12);
		file->print("gyroYcal,");
		file->println(gyroYcal, 12);
		file->print("gyroZcal,");
		file->println(gyroZcal, 12);
		file->print("altitudeCal,");
		file->println(altitudeCal, 12);
		file->print("temperature,");
		file->println(temperature, 12);
		file->println("END SENSOR CALIBRATION");
		file->flush();
	}
	
	void SteamEngineAHRS::resetCalibrationMetrics() {
		accXcal = 0.0;
		accYcal = 0.0;
		accZcal = 0.0;
		gyroXcal = 0.0;
		gyroYcal = 0.0;
		gyroZcal = 0.0;
		altitudeCal = 0.0;
		calibrationCount = 0;
		calibrated = false;
	}

	void SteamEngineAHRS::calibrate() {
		if (!calibrated) {
			//trap prior accelerometer values
			float ax = accX;
			float ay = accY;
			float az = accZ;
			//update to current values
			update();
			//compare prior to current values
			if (abs(accX) < ACCEL_JITTER && abs(accY) < ACCEL_JITTER && abs(accZ - SENSORS_GRAVITY_EARTH) < ACCEL_JITTER) {
				if (calibrationCount >= MIN_CALIBRATION_COUNT) {
					accXcal /= (float) calibrationCount;
					accYcal /= (float) calibrationCount;
					accZcal /= (float) calibrationCount;
					gyroXcal /= (float) calibrationCount;
					gyroYcal /= (float) calibrationCount;
					gyroZcal /= (float) calibrationCount;
					altitudeCal /= (float) calibrationCount;      
					calibrated = true;
				} else {
					accXcal += accX;
					accYcal += accY;
					accZcal += accZ - G;
					gyroXcal += gyroX;
					gyroYcal += gyroY;
					gyroZcal += gyroZ;
					altitudeCal += altitude;
					calibrationCount++;
				}			
			} else {    
				resetCalibrationMetrics();
			}
		}
	}
	  
	boolean SteamEngineAHRS::isCalibrated() {return calibrated;}
	unsigned long SteamEngineAHRS::getTimestamp() {return timestamp;}
	float SteamEngineAHRS::getTimespan() {return timespan;}
	float SteamEngineAHRS::getTemperature() {return temperature;}
	float SteamEngineAHRS::getAccX() {return accX - accXcal;}
	float SteamEngineAHRS::getAccY() {return accY - accYcal;}
	float SteamEngineAHRS::getAccZ() {return accZ - accZcal;}
	float SteamEngineAHRS::getGyroX() {return gyroX - gyroXcal;}
	float SteamEngineAHRS::getGyroY() {return gyroY - gyroYcal;}
	float SteamEngineAHRS::getGyroZ() {return gyroZ - gyroZcal;}
	float SteamEngineAHRS::getAltitude() {return altitude - altitudeCal;}

