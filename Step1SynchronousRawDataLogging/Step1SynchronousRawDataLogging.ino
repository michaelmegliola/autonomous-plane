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

/*
 * Represents a single, complete set of sensor readings at a specified point in time (to the millisecond).
 */
class SensorReadings {
  
  const float JITTER = 0.10;
  
  public: 
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

  private:
  Adafruit_BMP280 *barometer;
  Adafruit_FXAS21002C *gyro;
  Adafruit_FXOS8700 *accelmag;
  sensors_event_t* gyroEvent;
  sensors_event_t* accelEvent;
  sensors_event_t* magEvent;

  public:
  SensorReadings(Adafruit_BMP280 *b, Adafruit_FXAS21002C *g, Adafruit_FXOS8700* a) {
    barometer = b;
    gyro = g;
    accelmag = a;
    gyroEvent = new sensors_event_t();
    accelEvent = new sensors_event_t();
    magEvent = new sensors_event_t();
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

  boolean moved(SensorReadings *prior) {
    boolean r = false;
    r  = abs(accX - prior->accX) > JITTER;
    r |= abs(accY - prior->accY) > JITTER;
    r |= abs(accZ - prior->accZ) > JITTER;
    return r;
  }
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
    i = 0;
  }
  
  void update(SensorReadings *sensorReadings) {
    accXbuffer[i] = sensorReadings->accX;
    accYbuffer[i] = sensorReadings->accY;
    accZbuffer[i] = sensorReadings->accZ;
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

// sensors
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
Adafruit_FXAS21002C gyro(0x0021002C);
Adafruit_FXOS8700 accelmag(0x8700A, 0x8700B);

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
  initializeSensorArray();
  readings = new SensorReadings(&bmp, &gyro, &accelmag);
  accFilter = new AccLowPassFilter(ACC_FILTER_BUFFER_SIZE);
  datalog = getNextAvailableFileHandle();
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
    if (blink) digitalWrite(GREEN_LED, LOW);
    else digitalWrite(GREEN_LED, HIGH);
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

void terminalError() {
  while(true) {
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, HIGH);
    delay(50);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, LOW); 
    delay(50); 
  }
}

void initializeSdCard() {
  if (!SD.begin(CHIP_SELECT)) {
    terminalError();
  }
}

void initializeSensorArray() {
  if (!bmp.begin()) {  
    terminalError();
  }
  if(!gyro.begin(GYRO_RANGE_500DPS)) {
    terminalError();
  }
  if(!accelmag.begin(ACCEL_RANGE_4G)) {
    terminalError();
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
  datalog.print(readings->timestamp);
  datalog.print(",");
  datalog.print(readings->altitude);
  datalog.print(",");
  datalog.print(readings->temperature);
  datalog.print(",");
  datalog.print(readings->gyroX, 12);
  datalog.print(",");
  datalog.print(readings->gyroY, 12);
  datalog.print(",");
  datalog.print(readings->gyroZ, 12);
  datalog.print(",");
  datalog.print(readings->accX, 6);
  datalog.print(",");
  datalog.print(readings->accY, 6);
  datalog.print(",");
  datalog.println(readings->accZ, 6);
  datalog.print(",");
  datalog.println(accFilter->getAccX(), 6);
  datalog.print(",");
  datalog.println(accFilter->getAccY(), 6);
  datalog.print(",");
  datalog.println(accFilter->getAccZ(), 6);
}

