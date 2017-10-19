/*
 * Step 1. Synchronous raw data logging.
 * 
 * This sketch logs data to and SDI (Micro SD) file as quickly
 * as possible. There is no timer or synchronization... the loop
 * simply takes sensor readings and writes raw, uncalibrated
 * results to a file.
 * 
 * The purpose of this step is to fly the aircraft by remote
 * control while collecting actual sensor readings. By aligning
 * those readings in time with a video of the flight, models can
 * be developed for (a) filters and (b) fusion algorithms.
 */
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <SPI.h>
#include <SD.h>

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024
#define ACC_FILTER_BUFFER_SIZE 4

#define BMP_SCK 10    // BMP280 pin assignments
#define BMP_MISO 11
#define BMP_MOSI 12 
#define BMP_CS 13

#define GREEN_LED 8   // Onboard LED
#define RED_LED 13    // Onboard LED
#define CHIP_SELECT 4 // SDI (micro SD memory card)

#define ACCEL_JITTER 0.20
#define MIN_CALIBRATION_COUNT 1000
/*
 * Represents a single, complete set of sensor readings at a specified point in time (to the millisecond).
 */
class SensorReadings {
  
  private: 

  //primary registers
  unsigned long timestamp; 
  float altitude;
  float temperature;
  float gyroX;
  float gyroY;
  float gyroZ;
  float accX;
  float accY;
  float accZ;
  float magX;
  float magY;
  float magZ;
  
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
  
  // sensors
  Adafruit_BMP280 *barometer;
  Adafruit_FXAS21002C *gyro;
  Adafruit_FXOS8700 *accelmag;
  
  sensors_event_t *gyroEvent;
  sensors_event_t *accelEvent;
  sensors_event_t *magEvent;

  public:
  SensorReadings() {
    barometer = new Adafruit_BMP280(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
    gyro = new Adafruit_FXAS21002C(0x0021002C);
    accelmag = new Adafruit_FXOS8700(0x8700A, 0x8700B);
    if (!(barometer->begin() && gyro->begin(GYRO_RANGE_500DPS) && accelmag->begin(ACCEL_RANGE_4G))) {
      terminalError();
    }
    gyroEvent = new sensors_event_t();
    accelEvent = new sensors_event_t();
    magEvent = new sensors_event_t();
    
    resetCalibrationMetrics();
  }

  void update() { 
    gyro->getEvent(gyroEvent);
    accelmag->getEvent(accelEvent, magEvent);
    timestamp = gyroEvent->timestamp;
    altitude = barometer->readAltitude(1013.25);
    temperature = barometer->readTemperature();
    gyroX = gyroEvent->gyro.x;
    gyroY = gyroEvent->gyro.y;
    gyroZ = gyroEvent->gyro.z;
    accX = accelEvent->acceleration.x;
    accY = accelEvent->acceleration.y;
    accZ = accelEvent->acceleration.z;
    magX = magEvent->magnetic.x;
    magY = magEvent->magnetic.y;
    magZ = magEvent->magnetic.z;
  }

  void terminalError() {
    while (true) {
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(RED_LED, LOW);
      delay(50);
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, HIGH); 
      delay(50); 
    }
  }

  void recalibrate() {
    unsigned long start = millis();
    boolean flash = false;
    while (!calibrated) {
      calibrate();
      if (millis() > start + 50) {   
        digitalWrite(RED_LED, flash ? HIGH : LOW);
        digitalWrite(GREEN_LED, flash ? HIGH : LOW);
        flash = !flash; 
        start = millis();
      }
    }
    digitalWrite(RED_LED, LOW);
  }

  void describe(File *file) {
    file->println("=== START SENSOR CALIBRATION ==================");
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
    file->println("=== END SENSOR CALIBRATION ====================");
    file->flush();
  }

  private:
  void resetCalibrationMetrics() {
    gyroXcal = 0.0;
    gyroYcal = 0.0;
    gyroZcal = 0.0;
    altitudeCal = 0.0;
    calibrationCount = 0;
    calibrated = false;
  }
  
  void calibrate() {
    if (!calibrated) {
      //trap prior accelerometer values
      float ax = accX;
      float ay = accY;
      float az = accZ;
      //update to current values
      update();
      //compare prior to current values
      if (abs(accX) < ACCEL_JITTER && abs(accY) < ACCEL_JITTER && abs(az - accZ) < ACCEL_JITTER) {
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
          accZcal += accZ;
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
  
  boolean isCalibrated() {
    return calibrated;
  }

  public:
  unsigned long getTimestamp() {return timestamp;}
  float getTemperature() {return temperature;}
  float getAccX() {return accX - accXcal;}
  float getAccY() {return accY - accYcal;}
  float getAccZ() {return accZ - accZcal;}
  float getGyroX() {return gyroX - gyroXcal;}
  float getGyroY() {return gyroY - gyroYcal;}
  float getGyroZ() {return gyroZ - gyroZcal;}
  float getAltitude() {return altitude - altitudeCal;}
};

class AccLowPassFilter {

  private:
  
  float *accXbuffer;
  float *accYbuffer;
  float *accZbuffer;
  int i;
  int size;

  public:
  
  AccLowPassFilter(int buffersize) {
    size = buffersize;
    accXbuffer = new float[size];
    accYbuffer = new float[size];
    accZbuffer = new float[size];
    for (int n = 0; n < size; n++) {
      accXbuffer[n] = 0.0;
      accYbuffer[n] = 0.0;
      accZbuffer[n] = 0.0;
    }
    i = 0;
  }
  
  void update(SensorReadings *sensorReadings) {
    accXbuffer[i] = sensorReadings->getAccX();
    accYbuffer[i] = sensorReadings->getAccY();
    accZbuffer[i] = sensorReadings->getAccZ();
    i = ++i % size;
  }
  
  float getAccX() {return getAverageValue(accXbuffer);}
  float getAccY() {return getAverageValue(accYbuffer);}
  float getAccZ() {return getAverageValue(accZbuffer);}

  private:
  float getAverageValue(float *vals) {
    float sum = 0.0;
    for (int n = 0; n < size; n++) sum += vals[n];
    return sum / (float) size;
  }
};

class GyroIntegrator {
  
  private:
  
  float ix;
  float iy;
  float iz;
  unsigned long timestamp;
  
  public:
  
  GyroIntegrator() {
    ix = 0.0;
    iy = 0.0;
    iz = 0.0;
  }

  void update(SensorReadings *sensorReadings) {
    //todo
  }
};

// data logging file
File datalog;

// Low pass filter for accelerometer data
SensorReadings *readings;
AccLowPassFilter *accFilter;

unsigned long start_time;
unsigned long blink_time;
boolean blink;

void setup() { 
  initializeLEDs();
  initializeSdCard();
  datalog = getNextAvailableFileHandle();
  readings = new SensorReadings();
  readings->recalibrate();
  readings->describe(&datalog);
  accFilter = new AccLowPassFilter(ACC_FILTER_BUFFER_SIZE);
  start_time = millis();
  blink_time = start_time;
  delay(100);
}

void loop() {
  readings->update();
  accFilter->update(readings);
  writeToFile();

  // blink green LED
  if (millis() > blink_time + 250) {
    digitalWrite(GREEN_LED, blink ? LOW : HIGH);
    blink = !blink;
    blink_time = millis();
    datalog.flush();
  }
}

void initializeLEDs() {
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
}

void initializeSdCard() {
  if (!SD.begin(CHIP_SELECT)) {
    readings->terminalError();
  }
}

String getFileName(int i) {
  String s = "DATA";
  s += i;
  s += ".csv";
  return s;
}

File getNextAvailableFileHandle() {
  int i = 0;
  while (SD.exists(getFileName(i))) {
    i++;
  }
  return SD.open(getFileName(i), FILE_WRITE);
}

void writeToFile() {
  datalog.print(readings->getTimestamp());
  datalog.print(",");
  datalog.print(readings->getAltitude());
  datalog.print(",");
  datalog.print(readings->getTemperature());
  datalog.print(",");
  datalog.print(readings->getGyroX(), 12);
  datalog.print(",");
  datalog.print(readings->getGyroY(), 12);
  datalog.print(",");
  datalog.print(readings->getGyroZ(), 12);
  datalog.print(",");
  datalog.print(readings->getAccX(), 6);
  datalog.print(",");
  datalog.print(readings->getAccY(), 6);
  datalog.print(",");
  datalog.print(readings->getAccZ(), 6);
  datalog.print(",");
  datalog.print(accFilter->getAccX(), 6);
  datalog.print(",");
  datalog.print(accFilter->getAccY(), 6);
  datalog.print(",");
  datalog.println(accFilter->getAccZ(), 6);
}

