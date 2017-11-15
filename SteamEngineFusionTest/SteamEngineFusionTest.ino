#include <Servo.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <SteamEngine_AHRS.h>

Servo s0, s1, s2;
float* p_accel;
float* d_accel;
float* p_gyro;
float* d_gyro;
float* i_gyro;

// sensors
Adafruit_BMP280 bmp;
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

// AHRS support
SteamEngineAHRS ahrs = SteamEngineAHRS(&accelmag, &gyro, &bmp, 8);

void error() {
  while (true) {
    digitalWrite(RED_LED, HIGH);
    delay(100);
    digitalWrite(RED_LED, LOW);
    delay(100);
  }
}

void setup() {
  
  s0.attach(10);
  s1.attach(11);
  s2.attach(12);

  setServo(s0, -1.0);
  setServo(s1, -1.0);
  setServo(s2, -1.0);
  
  delay(1000);

  setServo(s0, 1.0);
  setServo(s1, 1.0);
  setServo(s2, 1.0);

  delay(1000);

  setServo(s0, 0.0);
  setServo(s1, 0.0);
  setServo(s2, 0.0);
  
  if (!accelmag.begin(ACCEL_RANGE_8G)) error(); 
  if (!gyro.begin()) error();
  if (!bmp.begin()) error();
 
  ahrs.recalibrate();
}

void setServo(Servo s, float pos) {
  pos = ( pos >  1.0 ) ?  1.0 : pos;
  pos = ( pos < -1.0 ) ? -1.0 : pos;
  int ms = 1500 + ( ( 2300 - 700 ) / 2 ) * pos;
  s.writeMicroseconds(ms);
}

void loop() {
  digitalWrite(RED_LED, HIGH);
  ahrs.update();
  p_accel = ahrs.getAccel(FILTERED);
  d_accel = ahrs.getAccel(DERIVATIVE);
  p_gyro = ahrs.getGyro(CORRECTED);
  d_gyro = ahrs.getGyro(DERIVATIVE);
  i_gyro = ahrs.getGyro(INTEGRAL);
  /*
   * accelerometer only
   */
  setServo(s0, p_accel[Y] / 2.0);

  /*
   * gyroscope only
   */
  setServo(s1, i_gyro[X] / 5.0);

  /*
   * both sensors
   */
  //TODO. fusion
}
