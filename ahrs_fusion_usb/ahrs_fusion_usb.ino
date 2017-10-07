#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Mahony.h>
#include <Madgwick.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

// Note: This sketch is a WORK IN PROGRESS

// Create sensor instances.
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

/*
 * Uncalibrated! ...these values were provided in an example
 * from Adafruit.
 */
// Mag calibration values are calculated via ahrs_calibration.
// These values must be determined for each board/environment.
// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { 0.93F, -7.47F, -35.23F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.943,  0.011,  0.020 },
                                    {  0.022,  0.918, -0.008 },
                                    {  0.020, -0.008,  1.156 } };

float mag_field_strength        = 50.23F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

//Mahony filter; //lightweight; use on slower systems
Madgwick filter;

void setup()
{
  Serial.begin(115200);

  // Wait for the Serial Monitor to open (comment out to run without Serial Monitor)
  // while(!Serial);

  Serial.println(F("Adafruit AHRS Fusion Example")); Serial.println("");

  // Initialize the sensors.
  if(!gyro.begin())
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

  // Filter expects 70 samples per second
  // Based on a Bluefruit M0 Feather ... rate should be adjuted for other MCUs
  filter.begin(10);
}

void loop(void)
{
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
  Serial.print(millis());
  Serial.print(" - Orientation: ");
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(roll);

  delay(10);
}
