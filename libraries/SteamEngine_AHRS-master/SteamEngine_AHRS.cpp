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
		xyzGyro = new XyzCal(GYRO, false);
		xyzAccel = new XyzCal(ACCEL, true);
		reset();
	}

	void SteamEngineAHRS::update() { 

		if (_gyro != NULL) _gyro->getEvent(gyroEvent);
		if (_accel != NULL) _accel->getEvent(accelEvent);

		// use prior timestamp value to calculate time span...
		timespan = (float) (gyroEvent->timestamp - timestamp) / 1000.0;
		// ...then update timestamp to new value.
		timestamp = gyroEvent->timestamp;

		if (_bar != NULL) {
		//	elminated extreme readings from altimeter (unless time limit as elapsed). 
			float reading = _bar->readAltitude(SENSORS_PRESSURE_SEALEVELHPA) - altitudeCal;
			if (altTimestamp == 0) {
				altitude = reading;
				altTimestamp = timestamp;
				dAlt_dt = 0.0;
				altIsUpdated = true;
			} else if ( reading != altitude || timestamp > altTimestamp + ALT_TIME_LIMIT_MILLIS ) {  //altimeter may update slowly... ignore unchanged readings
				altTimespan = (float) ( timestamp - altTimestamp ) / 1000.0;
				dAlt_dt = ( reading - altitude ) / altTimespan;
				if ( (dAlt_dt > -2.5 * ALT_CLIMB_LIMIT_MPS && dAlt_dt < ALT_CLIMB_LIMIT_MPS) || timestamp > altTimestamp + ALT_TIME_LIMIT_MILLIS) {
			//		the alitimeter is returning valid reading (or time limit has elapsed)...
					altitude = reading;
					altTimestamp = timestamp;
					altIsUpdated = true;
				} else {
			//		the alitimeter is returning spurious readings...
					altIsUpdated = false;
				}
			}
			temperature = _bar->readTemperature();
		}
		
		// to save time and memory, there is only one xyz buffer, so use contents immediately
		fillXyz(gyroEvent, SENSOR_TYPE_GYROSCOPE);
		xyzGyro->update(xyz, timespan);

		fillXyz(accelEvent, SENSOR_TYPE_ACCELEROMETER);
		xyzAccel->update(xyz, timespan);
		
		//TODO: integrate gyro readings and fuse into roll calculation
		float* vec = xyzAccel->getXyz(FILTERED);
		roll = (float) atan2(vec[Y], vec[Z]);
		
		//TODO: integrate gyro readings and fuse into pitch calculation
		if (vec[Y] * sin(roll) + vec[Z] * cos(roll) == 0) {
			pitch = vec[X] > 0 ? (PI_F / 2) : (-PI_F / 2);
		} else {
			pitch = (float) atan(-vec[X] / (vec[Y] * sin(roll) + vec[Z] * cos(roll)));
		}
		
		// convert to degrees
		roll *= 180.0 / PI_F;
		pitch *= 180.0 / PI_F;
	}

	float SteamEngineAHRS::getDeltaAltitude() {
		return dAlt_dt;
	}
	
	float SteamEngineAHRS::getAltitudeCalibration() {
		return altitudeCal;
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
		// Calibrate accelerometer and gyroscope
		while (!calibrated) {
			//update to current values
			update();
			//compare prior to current values
			if (isApproximatelyLevel()) {
				if (calibrationCount >= MIN_CALIBRATION_COUNT) {
					xyzGyro->calibrate(calibrationCount);
					xyzAccel->calibrate(calibrationCount);
					calibrated = true;
				} else {
					fillXyz(gyroEvent, SENSOR_TYPE_GYROSCOPE);
					xyzGyro->accumulate(xyz);
					fillXyz(accelEvent, SENSOR_TYPE_ACCELEROMETER);
					xyzAccel->accumulate(xyz);
					calibrationCount++;
				}
			} else {
				reset();
				xyzGyro->reset();
				xyzAccel->reset();
				countdownFlash();
			}
		}
		// Calibrate altimeter
		bool altCalibrated = false;
		float* alts = new float[10];
		float max;
		float min;
		while (!altCalibrated) {
			altitudeCal = 0.0;
			for (int i = 0; i < 10; i++) {
				alts[i] = _bar->readAltitude(SENSORS_PRESSURE_SEALEVELHPA);
				max = ( i == 0 || alts[i] > max ) ? alts[i] : max;
				min = ( i ==0 || alts[i] < min ) ? alts[i] : min;
				delay(20);
			}
			if ( max - min < 1.0 ) {
				for (int i = 0; i < 10; i++) {
					altitudeCal += alts[i];
				}
				altitudeCal /= 10.0;
				altCalibrated = true;			
			}	
		}
		delete(alts);
		altTimestamp = 0;
		// Prepare accelerometer & gyroscope for calibrated use
		xyzGyro->postCalibrate();
		xyzAccel->postCalibrate();
	}

	void SteamEngineAHRS::reset() {
		calibrationCount = 0;
		calibrated = false;
		altitudeCal = 0.0;
		altTimestamp = 0;
		altIsUpdated = false;
	}
	
	bool SteamEngineAHRS::isApproximatelyLevel() {
		float* vec = xyzAccel->getXyz(RAW);
		return vec[Z] > SENSORS_GRAVITY_EARTH - 0.50 && vec[Z] < SENSORS_GRAVITY_EARTH + 0.50;
	}
	  
	bool SteamEngineAHRS::isCalibrated() {return calibrated;}
	unsigned long SteamEngineAHRS::getTimestamp() {return timestamp;}
	float SteamEngineAHRS::getTimespan() {return timespan;}
	float SteamEngineAHRS::getTemperature() {return temperature;}
	float SteamEngineAHRS::getAltitude() {return altitude;}
	float* SteamEngineAHRS::getGyro(XyzType type) {return xyzGyro->getXyz(type);} 
	float* SteamEngineAHRS::getAccel(XyzType type) {return xyzAccel->getXyz(type);}
	float SteamEngineAHRS::getStaticPitch() {return pitch;}
	float SteamEngineAHRS::getStaticRoll() {return roll;}
	float SteamEngineAHRS::getAltitudeRaw() {return _bar == NULL ? 0.0 : _bar->readAltitude(SENSORS_PRESSURE_SEALEVELHPA);}
	
	void SteamEngineAHRS::logHeader(File* file) {
		file->print("TIME,TIMESPAN,ALTITUDE_RAW,ALTITUDE,ALT_TIMESPAN,D_ALT_D_T,TEMPERATURE,");
		xyzAccel->logHeader(file);
		xyzGyro->logHeader(file);
		file->println("");
	}
	
	void SteamEngineAHRS::log(File* file) {
		file->print(timestamp);
		file->print(",");
		file->print(timespan, 12);
		file->print(",");
		file->print(getAltitudeRaw(), 3);
		file->print(",");
		file->print(altitude, 3);
		file->print(",");
		file->print(altTimespan, 7);
		file->print(",");
		file->print(dAlt_dt, 7);
		file->print(",");
		file->print(temperature, 3);
		file->print(",");
		xyzAccel->log(file);
		xyzGyro->log(file);
		file->println("");
	}


