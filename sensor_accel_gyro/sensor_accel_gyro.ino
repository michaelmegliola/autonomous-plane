
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <SPI.h>
#include <SD.h>

// sensors
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

    /* Initialise the gyro sensor */
  if(!gyro.begin(GYRO_RANGE_250DPS))
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
  if (millis() > start + 1000) {
    datalog.flush();
    start = millis();
  }

  /* Get a new sensor event */
  sensors_event_t gyroEvent;
  gyro.getEvent(&event);
  accelmag.getEvent(&event);
    
  // make a string for assembling the data to log:
  String dataString = "";
  dataString += count;
  dataString += ",";
  dataString += micros();
  dataString += ",";
  dataString += gyroEvent.gyro.x;  
  dataString += ",";
  dataString += gyroEvent.gyro.y;  
  dataString += ",";
  dataString += gyroEvent.gyro.z;
  dataString += ",";
  dataString += accelmag.acceleration.x;
  dataString += ",";
  dataString += accelmag.acceleration.y;
  dataString += ",";
  dataString += accelmag.acceleration.z;
  // if the file is available, write to it:
  if (datalog) {
    datalog.println(dataString); 
  }
}









