
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
SteamEngineAHRS* ahrs;

// file for data logging
File logfile;

void setup() {

  if (!SD.begin(4)) {
    Serial.println("Card failed, or not present");
    return;
  }

  logfile = SD.open("SteamEngineTest.csv", FILE_WRITE);  // open the logfile for writing
  if( ! logfile ) {
    Serial.print("Could not create file.");
  }
  
  if(!accelmag.begin())
  {
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(1);
  }  

  Serial.println("Gyroscope Test"); Serial.println("");
  if(!gyro.begin())
  {
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while(1);
  }

  if (!bmp.begin()) {
    Serial.println("Ooops, no BMP280 detected ... Check your wiring!");
    while(1);
  }

  ahrs = new SteamEngineAHRS(&accelmag, &gyro, &bmp, 8);
  ahrs->recalibrate();
}

void loop() {
  ahrs->update();
}
