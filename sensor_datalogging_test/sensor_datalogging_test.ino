
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <SPI.h>
#include <SD.h>

/*
 * Pin assignments for BMP280 temperature and pressure sensor
 */
#define BMP_SCK 10
#define BMP_MISO 11
#define BMP_MOSI 12 
#define BMP_CS 13

// sensors
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

// SDI (microSD memory card)
const int chipSelect = 4;

/*
 * Data logging file
 */
File datalog;

int count = 0;
unsigned long start;

String getFileName(int i) {
  String s = "DATA";
  s += i;
  s += ".txt";
  return s;
}

File getNextAvailableFileHandle() {
  int i = 0;
  while (SD.exists(getFileName(i))) {
    Serial.println(getFileName(i));
    i++;
  }
  return SD.open(getFileName(i), FILE_WRITE);
}

void setup() {
  
  // open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Starting data logging test.");
  
 // initialize SD memory card
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while(true);
  }
  datalog = getNextAvailableFileHandle();

  // initialize BMP temperature & pressure sensor
  if (!bmp.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (true);
  }

    /* Initialise the sensor @ 2000 degrees per second */
  if(!gyro.begin(GYRO_RANGE_2000DPS))
  {
    /* There was a problem detecting the FXAS21002C ... check your connections */
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while(true);
  }

  /* Initialise the sensor */
  if(!accelmag.begin(ACCEL_RANGE_4G))
  {
    /* There was a problem detecting the FXOS8700 ... check your connections */
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(true);
  }

  start = millis();
}

void loop() {

  count++;
  if (millis() > start + 10000) {
    datalog.flush();
    datalog.close();
    Serial.print("Completed: ");
    Serial.println(count);
    while(true);
  }

  /* Get a new sensor event */
  sensors_event_t event;
  gyro.getEvent(&event);
  accelmag.getEvent(&event);
    
  // make a string for assembling the data to log:
  String dataString = "";
  dataString += count;
  dataString += ",";
  dataString += micros();
  dataString += ",";
  dataString += bmp.readTemperature();
  dataString += ",";
  dataString += bmp.readAltitude(1013.25);
  dataString += ",";
  dataString += event.gyro.x;  
  dataString += ",";
  dataString += event.gyro.y;  
  dataString += ",";
  dataString += event.gyro.z;
  dataString += ",";
  dataString += accelmag.accel_raw.x;
  dataString += ",";
  dataString += accelmag.accel_raw.y;
  dataString += ",";
  dataString += accelmag.accel_raw.z;
  // if the file is available, write to it:
  if (datalog) {
    datalog.println(dataString); 
  }
}









