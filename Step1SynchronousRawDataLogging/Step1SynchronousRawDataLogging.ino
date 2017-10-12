/*
 * Step 1. Raw data logging.
 * 
 * For the code that sets up the timer and callback function,
 * special thanks to Sheng Chen https://gist.github.com/jdneo
 */
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <SPI.h>
#include <SD.h>

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

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
  
  const float JITTER = 0.20;
  
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

  public:
  SensorReadings(unsigned long time, Adafruit_BMP280 bmp, sensors_event_t* gyroEvent, sensors_event_t* accelEvent, sensors_event_t* magEvent) {
    timestamp = time;
    altitude = bmp.readAltitude(1013.25);
    temperature = bmp.readTemperature();
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

  public:
  boolean moved(SensorReadings* prior) {
    boolean r = false;
    r  = abs(accX - prior->accX) > JITTER;
    r |= abs(accY - prior->accY) > JITTER;
    r |= abs(accZ - prior->accZ) > JITTER;
    return r;
  }

  public:
  String getDataString() {
    String data;
    data += timestamp;
    data += ",";
    data += altitude;
    data += ",";
    data += temperature;
    data += ",";
    data += gyroX;
    data += ",";
    data += gyroY;  
    data += ",";
    data +=  gyroZ;
    data += ",";
    data +=  accX;
    data += ",";
    data +=  accY;
    data += ",";
    data +=  accZ;
    data += ",";
    data +=  magX;
    data += ",";
    data +=  magY;
    data += ",";
    data +=  magZ;
    return data;
  }
};

// sensors
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

// data logging file
File datalog;

uint16_t prior_event_time;
uint16_t start_time;

int countNoMovement;
SensorReadings* priorReadings;

void setup() {  
  Serial.begin(9600);
  while(!Serial);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  initializeSdCard();
  initializeSensorArray();
  datalog = getNextAvailableFileHandle();
  countNoMovement = 0;
  priorReadings = getSensorReadings();
  delay(100);
}

void loop() {
  SensorReadings* readings = getSensorReadings();
  datalog.println(readings->getDataString());
  if (readings->moved(priorReadings)) {
    countNoMovement = 0;
  } else {
    countNoMovement++;
  }
  Serial.println(countNoMovement);
  delete priorReadings;
  priorReadings = readings;
  if (countNoMovement > 1000) {
    datalog.flush();
    datalog.close();
    while (true) {
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(GREEN_LED, HIGH);
    }
  }
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
  if(!gyro.begin(GYRO_RANGE_250DPS)) {
    terminalError();
  }
  if(!accelmag.begin(ACCEL_RANGE_4G)) {
    terminalError();
  }
}

String getFileName(int i) {
  String s = "DATA";
  s += i;
  s += ".txt";
  return s;
}

File getNextAvailableFileHandle() {
  int i = 0;
  while (SD.exists(getFileName(i))) {
    i++;
  }
  return SD.open(getFileName(i), FILE_WRITE);
}

SensorReadings* getSensorReadings() {
  sensors_event_t gyroEvent, accelEvent, magEvent;
  gyro.getEvent(&gyroEvent);
  accelmag.getEvent(&accelEvent, &magEvent);
  return new SensorReadings(millis(), bmp, &gyroEvent, &accelEvent, &magEvent);
}

