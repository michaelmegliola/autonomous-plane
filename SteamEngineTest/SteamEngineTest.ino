
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

void setup() {
  Serial.begin(9600);
  while (!Serial) {}
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

  Serial.println("STARTING...");
  ahrs = new SteamEngineAHRS(&accelmag, &gyro, NULL, 8);

  Serial.println("Recalibrating...");
  ahrs->recalibrate();
  iGyro = 0.0;
  flashtime = millis();
}

void loop() {
  ahrs->update();

  if (millis() > flashtime + 5000) {
    digitalWrite(RED_LED, flash ? HIGH : LOW);
    flash = !flash;
    flashtime = millis();
  }

  if (ahrs->isApproximatelyLevel()) {
    iGyro = 0.0;
  } else {
    iGyro += ahrs->getGyro(CORRECTED)[X];
  }
  
Serial.print(ahrs->getAccel(RAW)[Z], 12);
Serial.print(" ");
Serial.print(ahrs->getAccel(CORRECTED)[Z], 12);
Serial.print(" ");
Serial.print(ahrs->isApproximatelyLevel());
Serial.print("    ");
  Serial.print(ahrs->getGyro(RAW)[X], 12);
  Serial.print(" ");
  Serial.print(ahrs->getGyro(CALIBRATION)[X], 12);
  Serial.print(" ");
  Serial.print(ahrs->getGyro(CORRECTED)[X], 12);
  Serial.print(" ");
  Serial.println(iGyro, 12);
}