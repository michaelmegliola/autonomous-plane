#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>

const int chipSelect = 4;
unsigned long interval = 1000;
unsigned long previoustime = 0;
File logfile;
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
float gyroXc = 0.0;
float gyroYc = 0.0;
float gyroZc = 0.0;
int count = 0;

void setup() {
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
Serial.begin(115200);  

  for (int i = 0; i < 10; i++) {
    digitalWrite(8, HIGH);
    delay(500);
    digitalWrite(8, LOW);
    delay(500);
  }

//while (!Serial) {
//    delay(1);
//}
      
Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  char filename[15];
  strcpy(filename, "jeflog00.csv");
  for (byte i = 0; i < 100; i++) {  // Check if the filename has already been used, iterate till free name found
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  logfile = SD.open(filename, FILE_WRITE);  // open the logfile for writing
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
  }
  
  Serial.println("FXOS8700 Test"); Serial.println("");
  if(!accelmag.begin(ACCEL_RANGE_4G))
  {
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(1);
  }  

  Serial.println("Gyroscope Test"); Serial.println("");
  if(!gyro.begin())
  {
    /* There was a problem detecting the FXAS21002C ... check your connections */
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while(1);
  }
}

void loop() {

  sensors_event_t aevent, mevent;
  sensors_event_t event;
  gyro.getEvent(&event);
//unsigned long currenttime = millis();
//  if (currenttime - previoustime >= interval) {
//    previoustime = currenttime;  
   if (count < 1000) {
    gyroXc += event.gyro.x;
    gyroYc += event.gyro.y;
    gyroZc += event.gyro.z; 
   } else if (count == 1000) {
    gyroXc /= 1000.0;
    gyroYc /= 1000.0;
    gyroZc /= 1000.0;
   } else {
    digitalWrite(8, HIGH);
    logfile.print(millis() );
    logfile.print(" ");
    accelmag.getEvent(&aevent, &mevent);
    logfile.print(aevent.acceleration.x, 8); 
    logfile.print(",");
    logfile.print(aevent.acceleration.y, 8); 
    logfile.print(",");
    logfile.print(aevent.acceleration.z, 8); 
    logfile.print(" ,");
//  for (byte i = 0; i < 10; i++) {
//  logfile.print("writing line number");
//  logfile.println(i);  
//  }

    logfile.print(event.gyro.x-gyroXc, 8); 
    logfile.print(","); 
    logfile.print(event.gyro.y-gyroYc, 8); 
    logfile.print(","); 
    logfile.println(event.gyro.z-gyroZc, 8); 
    logfile.flush();
    }
//  }
  count++;

}
