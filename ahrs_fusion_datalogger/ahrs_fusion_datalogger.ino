#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Mahony.h>
#include <Madgwick.h>
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

//#define LOG_MODE 1
#define SERIAL_MODE 1
//#define LED_MODE 1

// Create sensor instances.
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

// Mag calibration values are calculated via ahrs_calibration.
// These values must be determined for each board/environment.
// Offsets applied to raw x/y/z mag values

/*
 * Initial calibration including all metal masses (battery, motor),
 * 
 */
float mag_offsets[3]            = { 11.39F, -2.08F, 154.79F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.990,  0.004,  0.004 },
                                    {  0.004,  0.990,  0.001 },
                                    {  0.004,  0.001,  1.020 } };

float mag_field_strength        = 46.21F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

//Mahony filter; //lightweight; use on slower systems
Madgwick filter;

/*
 * Data logging file
 */
File datalog;
const int chipSelect = 4;
int count = 0;
unsigned long start;
int flicker;

String getFileName(int i) {
  String s = "DATA";
  s += i;
  s += ".txt";
  return s;
}

File getNextAvailableFileHandle() {
  int i = 0;
  Serial.println(getFileName(i));
  while (SD.exists(getFileName(i))) {
    Serial.println(getFileName(i));
    i++;
  }
  return SD.open(getFileName(i), FILE_WRITE);
}

void setup()
{
  // built in green LED 
  pinMode(8, OUTPUT);
  for (int i = 0; i < 10; i++) {
    digitalWrite(8, HIGH);
    delay(200);
    digitalWrite(8, LOW);
    delay(200);
  }
  Serial.begin(115200);

  #ifdef SERIAL_MODE
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
  #endif
  
  Serial.println(F("Sensor & datalogging test."));

  // Initialize the sensors.
  if(!gyro.begin(/*GYRO_RANGE_2000DPS9*/))
  {
    /* There was a problem detecting the gyro ... check your connections */
    Serial.println("Ooops, no gyro detected ... Check your wiring!");
    while(1);
  }

  if(!accelmag.begin(ACCEL_RANGE_4G))
  {
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(1);
  }

  // initialize BMP temperature & pressure sensor
  if (!bmp.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (true);
  }

  // Based on a Bluefruit M0 Feather ... rate should be adjuted for other MCUs
  filter.begin(75);

  #ifdef LOG_MODE
    // initialize SD memory card
    if (!SD.begin(chipSelect)) {
      Serial.println("Card failed, or not present");
      // don't do anything more:
      while(true);
    }
    datalog = getNextAvailableFileHandle();
  
    if (!datalog) {
      Serial.println("Cannot write data file.");
      while(true);
    }
  #endif

  start = millis();
}

void loop(void)
{
  count++;
  
  #ifdef LOG_MODE  
    if (millis() > start + 60 * 1000) {
      datalog.flush();
      datalog.close();
      Serial.print("Completed: ");
      Serial.println(count);
      while(true) {
        digitalWrite(8, HIGH);
        delay(250);
        digitalWrite(8, LOW);
        delay(400);
      }
    }
  #endif
  
  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;

  // Get new data samples
  gyro.getEvent(&gyro_event);
  accelmag.getEvent(&accel_event, &mag_event);

  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
  float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
  float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

  // The filter library expects gyro data in degrees/s, but adafruit sensor
  // uses rad/s so we need to convert them first (or adapt the filter lib
  // where they are being converted)
  gx *= 57.2958F;
  gy *= 57.2958F;
  gz *= 57.2958F;

  // Update the filter
  
  filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);

  //filter.updateIMU(gx, gy, gz, accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z);

  // Print the orientation filter output
  // Note: To avoid gimbal lock you should read quaternions not Euler
  // angles, but Euler angles are used here since they are easier to
  // understand looking at the raw values. See the ble fusion sketch for
  // and example of working with quaternion data.

  /*
   * WIKIPEDIA (2017-10-06)
   * 
   * Unit quaternions, also known as versors, provide a convenient mathematical 
   * notation for representing orientations and rotations of objects in three 
   * dimensions. Compared to Euler angles they are simpler to compose and avoid 
   * the problem of gimbal lock. Compared to rotation matrices they are more 
   * compact, more numerically stable, and may be more efficient.
   * 
   * Gimbal lock is the loss of one degree of freedom in a three-dimensional, 
   * three-gimbal mechanism that occurs when the axes of two of the three gimbals 
   * are driven into a parallel configuration, "locking" the system into rotation 
   * in a degenerate two-dimensional space. The word lock is misleading: no gimbal 
   * is restrained. All three gimbals can still rotate freely about their respective 
   * axes of suspension. Nevertheless, because of the parallel orientation of two 
   * of the gimbals' axes there is no gimbal available to accommodate rotation along 
   * one axis.
   */
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float heading = filter.getYaw();

  #ifdef LOG_MODE
    //if (count % 100 == 0) {
      // make a string for assembling the data to log:
      String dataString = "";
      dataString += count;
      dataString += ",";
      dataString += micros();
      dataString += ",";
      dataString += heading;  
      dataString += ",";
      dataString += pitch;  
      dataString += ",";
      dataString += roll;
      dataString += ",";
      dataString += accel_event.acceleration.x;
      dataString += ",";
      dataString += accel_event.acceleration.y;
      dataString += ",";
      dataString += accel_event.acceleration.z;
      dataString += ",";
      dataString += gyro_event.gyro.x;
      dataString += ",";
      dataString += gyro_event.gyro.y;
      dataString += ",";
      dataString += gyro_event.gyro.z;
      dataString += ",";
      dataString += mag_event.magnetic.x;
      dataString += ",";
      dataString += mag_event.magnetic.y;
      dataString += ",";
      dataString += mag_event.magnetic.z;
      if (datalog) {
        datalog.println(dataString); 
      }
    //}
  #endif

  #ifdef SERIAL_MODE
    if (count % 1000 == 0) {
      Serial.print(heading, 6); Serial.print(" "); 
      Serial.print(pitch, 6); Serial.print(" ");
      Serial.print(roll, 6); Serial.print(" | ");
      Serial.print(accel_event.acceleration.x, 6); Serial.print(" ");
      Serial.print(accel_event.acceleration.y, 6); Serial.print(" ");
      Serial.print(accel_event.acceleration.z, 6); Serial.print(" | ");
      Serial.print(gyro_event.gyro.x, 6); Serial.print(" ");
      Serial.print(gyro_event.gyro.y, 6); Serial.print(" ");
      Serial.print(gyro_event.gyro.z, 6); Serial.print(" | ");
      Serial.print(mag_event.magnetic.x, 6); Serial.print(" ");
      Serial.print(mag_event.magnetic.y, 6); Serial.print(" ");
      Serial.println(mag_event.magnetic.z, 6);
    }
  #endif

  #ifdef LED_MODE
    if (roll > 30.0 || roll < -30.0) {
      digitalWrite(8, HIGH);
    } else {
      if (flicker % 30 > 15) {
        digitalWrite(8, LOW);
      } else {
        digitalWrite(8, HIGH);
      }
      flicker++;
    }
  #endif

}
