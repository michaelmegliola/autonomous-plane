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
		accelEvent = new sensors_event_t;
		gyroEvent = new sensors_event_t;
		barEvent = new sensors_event_t;
		orientation = new sensors_vec_t;
		lpfAccX = new SteamEngineLPF(LOW_PASS_FILTER_SIZE);
		lpfAccY = new SteamEngineLPF(LOW_PASS_FILTER_SIZE);
		lpfAccZ = new SteamEngineLPF(LOW_PASS_FILTER_SIZE);
		lpfAltitude = new SteamEngineLPF(LOW_PASS_FILTER_SIZE);
		resetCalibrationMetrics();
	}

	void SteamEngineAHRS::update() { 
		_gyro->getEvent(gyroEvent);
		_accel->getEvent(accelEvent);
		timespan = (float) (gyroEvent->timestamp - timestamp) / 1000.0;
		timestamp = gyroEvent->timestamp;

		//TODO: apply low pass filter
		altitude = _bar->readAltitude(1013.25) - altitudeCal;
		temperature = _bar->readTemperature();
		gyroX = gyroEvent->gyro.x - gyroXcal;
		gyroY = gyroEvent->gyro.y - gyroYcal;
		gyroZ = gyroEvent->gyro.z - gyroZcal;

		// apply low-pass filters to calibrated accelerometer readings.		
		lpfAccX->update(accelEvent->acceleration.x - accXcal);
		accX = lpfAccX->getFilteredVal();
		
		lpfAccY->update(accelEvent->acceleration.y - accYcal);
		accY = lpfAccY->getFilteredVal();

		lpfAccZ->update(accelEvent->acceleration.z - accZcal);
		accZ = lpfAccZ->getFilteredVal();
		
		//TODO: integrate gyro readings and fuse into roll calculation
		roll = (float) atan2(accY, accZ);
		roll *= 180.0 / PI_F;
		//TODO: integrate gyro readings and fuse into pitch calculation
		if (accY * sin(roll) + accZ * cos(roll) == 0) {
			pitch = accX > 0 ? (PI_F / 2) : (-PI_F / 2);
		} else {
			pitch = (float) atan(-accX / (accY * sin(roll) + accZ * cos(roll)));
		}
		pitch *= 180.0 / PI_F;
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
		digitalWrite(led, LOW);
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
		iGyroX = 0.0;
		iGyroY = 0.0;
		iGyroZ = 0.0;
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
					iGyroX = 0.0;
					iGyroY = 0.0;
					iGyroZ = 0.0;
					calibrated = true;
				} else {
					accXcal += accX;
					accYcal += accY;
					accZcal += accZ - SENSORS_GRAVITY_EARTH;
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
	float SteamEngineAHRS::getAccX() {return accX;}
	float SteamEngineAHRS::getAccY() {return accY;}
	float SteamEngineAHRS::getAccZ() {return accZ ;}
	float SteamEngineAHRS::getGyroX() {return gyroX ;}
	float SteamEngineAHRS::getGyroY() {return gyroY;}
	float SteamEngineAHRS::getGyroZ() {return gyroZ;}
	float SteamEngineAHRS::getAltitude() {return altitude;}
	float SteamEngineAHRS::getPitch() {return pitch;}
	float SteamEngineAHRS::getRoll() {return roll;}


