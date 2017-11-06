
#include <SPI.h>
#include <SD.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <SteamEngine_AHRS.h>

// data logging file
File datalog;

// sensors
//Adafruit_BMP280 bmp;
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

// AHRS support
SteamEngineAHRS* ahrs;

// file for data logging
File logfile;

unsigned long flashtime;
bool flash;
double iGyro;

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

void error() {
  while (true) {
    digitalWrite(RED_LED, HIGH);
    delay(100);
    digitalWrite(RED_LED, LOW);
    delay(100);
  }
}

void setup() {
  if (!SD.begin(4)) {
    error();
  }

  logfile = getNextAvailableFileHandle();
  if( ! logfile ) {
    error();
  }
  
  if(!accelmag.begin())
  {
    error();
  }  

  if(!gyro.begin())
  {
    error();
  }

  ahrs = new SteamEngineAHRS(&accelmag, &gyro, NULL, 8);  
  ahrs->logHeader(&logfile);
  ahrs->recalibrate();
  flashtime = millis();
}

void loop() {
  ahrs->update();
  ahrs->log(&logfile);
  
  if (millis() > flashtime + 1500) {
    logfile.flush();
    digitalWrite(RED_LED, flash ? HIGH : LOW);
    flash = !flash;
    flashtime = millis();
  }

}
