/*  =============================================================================
 *  Sensor Readings
 *  
 *  Represents a single, complete set of sensor readings at a 
 *  specified point in time (to the nearest millisecond).
 */

#include "SteamEngine_AHRS.h"
 
	SteamEngineAHRS::SteamEngineAHRS(Adafruit_Sensor* accelerometer, Adafruit_Sensor* gyroscope, Adafruit_BMP280* barometer, int ledPin) {
		_accel = accelerometer;
		_gyro = gyroscope;
		_bar = barometer;
		led = ledPin;
		// NOTE: the sensor events must be separate, because the Adafruit
		// library clears the entire data structure on update.
		accelEvent = new sensors_event_t;
		gyroEvent = new sensors_event_t;
		barEvent = new sensors_event_t;
		xyzGyro = new XyzCal(GYRO);
		xyzAccel = new XyzCal(ACCEL);
		calibrationCount = 0;
		calibrated = false;
	}

	void SteamEngineAHRS::update() { 
		if (_gyro != NULL) _gyro->getEvent(gyroEvent);
		if (_accel != NULL) _accel->getEvent(accelEvent);

		// use prior timestamp value to calculate time span...
		timespan = (float) (gyroEvent->timestamp - timestamp) / 1000.0;
		// ...then update timestamp to new value.
		timestamp = gyroEvent->timestamp;

		//TODO: apply low pass filter?
		if (_bar != NULL) {
			altitude = _bar->readAltitude(SENSORS_PRESSURE_SEALEVELHPA) - altitudeCal;
			temperature = _bar->readTemperature();
		}
		
		// to save time and memory, there is only one xyz buffer, so use contents immediately
		fillXyz(gyroEvent, SENSOR_TYPE_GYROSCOPE);
		xyzGyro->update(xyz);

		fillXyz(accelEvent, SENSOR_TYPE_ACCELEROMETER);
		xyzAccel->update(xyz);
		
		//TODO: integrate gyro readings and fuse into roll calculation
		float* vec = xyzAccel->getXyz(FILTERED);
		roll = (float) atan2(vec[Y], vec[Z]);
		roll *= 180.0 / PI_F;
		//TODO: integrate gyro readings and fuse into pitch calculation
		if (vec[Y] * sin(roll) + vec[Z] * cos(roll) == 0) {
			pitch = vec[X] > 0 ? (PI_F / 2) : (-PI_F / 2);
		} else {
			pitch = (float) atan(-vec[X] / (vec[Y] * sin(roll) + vec[Z] * cos(roll)));
		}
		pitch *= 180.0 / PI_F;
	}

	void SteamEngineAHRS::fillXyz(sensors_event_t* event, sensors_type_t type) {
		switch(type) {
			case SENSOR_TYPE_ACCELEROMETER:
				xyz[X] = event->acceleration.x;
				xyz[Y] = event->acceleration.y;
				xyz[Z] = event->acceleration.z;
			break;
			
			case SENSOR_TYPE_GYROSCOPE:
				xyz[X] = event->gyro.x;
				xyz[Y] = event->gyro.y;
				xyz[Z] = event->gyro.z;
			break;
		}
	}
	
	void SteamEngineAHRS::countdownFlash() {
		for (int i = 10; i > 0; i--) {
			digitalWrite(RED_LED, HIGH);
			delay(i * 100);
			digitalWrite(RED_LED, LOW);
			delay(i * 100);
		}
	}

	void SteamEngineAHRS::recalibrate() {
		countdownFlash();
		while (!calibrated) {
			//update to current values
			update();
			//compare prior to current values
			if (isApproximatelyLevel()) {
				if (calibrationCount >= MIN_CALIBRATION_COUNT) {
					xyzGyro->calibrate(calibrationCount);
					xyzAccel->calibrate(calibrationCount);  
					iGyroX = 0.0;
					iGyroY = 0.0;
					iGyroZ = 0.0;
					calibrated = true;
				} else {
					fillXyz(gyroEvent, SENSOR_TYPE_GYROSCOPE);
					xyzGyro->accumulate(xyz);
					fillXyz(accelEvent, SENSOR_TYPE_ACCELEROMETER);
					xyzAccel->accumulate(xyz);
					calibrationCount++;
				}
			} else {
				calibrationCount = 0;
				xyzGyro->reset();
				xyzAccel->reset();
				countdownFlash();
			}
		}
	}
	
	bool SteamEngineAHRS::isApproximatelyLevel() {
		return xyzAccel->isApproximatelyLevel();
	}
	  
	bool SteamEngineAHRS::isCalibrated() {return calibrated;}
	unsigned long SteamEngineAHRS::getTimestamp() {return timestamp;}
	float SteamEngineAHRS::getTimespan() {return timespan;}
	float SteamEngineAHRS::getTemperature() {return temperature;}
	float SteamEngineAHRS::getAltitude() {return altitude;}
	float* SteamEngineAHRS::getGyro(XyzType type) {return xyzGyro->getXyz(type);} 
	float* SteamEngineAHRS::getAccel(XyzType type) {return xyzAccel->getXyz(type);}
	float SteamEngineAHRS::getPitch() {return pitch;}
	float SteamEngineAHRS::getRoll() {return roll;}


