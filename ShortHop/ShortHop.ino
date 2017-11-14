
#include <SPI.h>
#include <SD.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <SteamEngine_AHRS.h>

// data logging file
File datalog;

// sensors
Adafruit_BMP280 bmp;
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

// AHRS support
SteamEngineAHRS ahrs = SteamEngineAHRS(&accelmag, &gyro, &bmp, 8);

// file for data logging
File logfile;

enum FlightMode { ABORT, PREFLIGHT, TAKEOFF, LANDING };
FlightMode mode;

float targetAlt;

void error() {
  while (true) {
    digitalWrite(RED_LED, HIGH);
    delay(100);
    digitalWrite(RED_LED, LOW);
    delay(100);
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

void setup() {
  if (!SD.begin(4)) {
    error();
  }

  logfile = getNextAvailableFileHandle();
  if (!logfile) error();
  if (!accelmag.begin(ACCEL_RANGE_8G)) error(); 
  if (!gyro.begin()) error();
  if (!bmp.begin()) error();
 
  ahrs.logHeader(&logfile);
  ahrs.recalibrate();
  mode = PREFLIGHT;
}

void loop() {
  
  //always update sensors
  ahrs.update();
  
  //excute flight mode
  switch (mode) {
    case ABORT:
    abort();
    break;
    
    case PREFLIGHT:
    break;
    
    case TAKEOFF:
    break;
    
    case LANDING:
    break;  
  }
  ahrs.log(&logfile);
}

void abort() {
  setThrottle(0.0);
}

void takeoff() {
  
}

void landing() {
  setTargetAltitude(0.0);
  proceed();
}

void proceed() {
  if (getAltitude() < 10.0) {
  }
}
}

