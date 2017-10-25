
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024
#define ACC_FILTER_BUFFER_SIZE 4
#define SERVO_MIN_MICROSECONDS 1000
#define SERVO_MAX_MICROSECONDS 2000

#define BMP_SCK 10    // BMP280 pin assignments
#define BMP_MISO 11
#define BMP_MOSI 12 
#define BMP_CS 13

#define ESC_PIN 5        // Electronic speed control
#define ELEVATOR_PIN 6   // Elevator servo
#define AILERON_PIN 7    // Aileron servo

#define GREEN_LED 8   // Onboard LED
#define RED_LED 13    // Onboard LED
#define CHIP_SELECT 4 // SDI (micro SD memory card)

#define ACCEL_JITTER 0.50           //maximum acceptable deviation for calibration
#define G -9.80665                  //acceleration of gravity (minus sign = downward)
#define MIN_CALIBRATION_COUNT 1000  //minimum consecutive calibration loops
#define RADIANS_TO_DEGREES 57.2958  //conversion factor for Gyro (library is in radians)

/*
 * Flight modes in order of precedence
 */
const int FM_TERMINATE           = 0x0001;
const int BL_PREFLIGHT_CALIBRATE = 0x0002;  // BL prefix indicates blocking code
const int BL_PREFLIGHT_ARM       = 0x0004;  // BL prefix indicates blocking code
const int FM_TAKEOFF             = 0x0008;
const int FM_LAND_IMMEDIATELY    = 0x0016;
const int FM_MAINTAIN_ALTITUDE   = 0x0032;
const int FM_RECALIBRATE_AHRS    = 0x0064;
const int FM_SEEK_WAYPOINT       = 0x0128;
const int FM_LOITER              = 0x0256;

/*
 * Represents a single, complete set of sensor readings at a 
 * specified point in time (to the nearest millisecond).
 */
class SensorReadings {
  
  private: 

  //primary registers
  unsigned long timestamp; 
  float timespan;
  float altitude;
  float temperature;
  float gyroX;
  float gyroY;
  float gyroZ;
  float accX;
  float accY;
  float accZ;
  float magX;
  float magY;
  float magZ;
  
  //dynamically measured calibration constants
  float accXcal;
  float accYcal;
  float accZcal;
  float gyroXcal;
  float gyroYcal;
  float gyroZcal;
  float altitudeCal;
  
  int calibrationCount;
  boolean calibrated;
  
  // sensors
  Adafruit_BMP280 *barometer;
  Adafruit_FXAS21002C *gyro;
  Adafruit_FXOS8700 *accelmag;
  
  sensors_event_t *gyroEvent;
  sensors_event_t *accelEvent;
  sensors_event_t *magEvent;

  public:
  SensorReadings() {
    barometer = new Adafruit_BMP280(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
    gyro = new Adafruit_FXAS21002C(0x0021002C);
    accelmag = new Adafruit_FXOS8700(0x8700A, 0x8700B);
    if (!(barometer->begin() && gyro->begin(GYRO_RANGE_500DPS) && accelmag->begin(ACCEL_RANGE_4G))) {
      terminalError();
    }
    gyroEvent = new sensors_event_t();
    accelEvent = new sensors_event_t();
    magEvent = new sensors_event_t();
    
    resetCalibrationMetrics();
  }

  void update() { 
    gyro->getEvent(gyroEvent);
    accelmag->getEvent(accelEvent, magEvent);
    timespan = (float) (gyroEvent->timestamp - timestamp) / 1000.0;
    timestamp = gyroEvent->timestamp;
    altitude = barometer->readAltitude(1013.25);
    temperature = barometer->readTemperature();
    gyroX = gyroEvent->gyro.x;
    gyroY = gyroEvent->gyro.y;
    gyroZ = gyroEvent->gyro.z;
    accX = accelEvent->acceleration.x;
    accY = accelEvent->acceleration.y;
    accZ = accelEvent->acceleration.z;
    magX = magEvent->magnetic.x;
    magY = magEvent->magnetic.y;
    magZ = magEvent->magnetic.z;
  }

  void terminalError() {
    while (true) {
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(RED_LED, LOW);
      delay(50);
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, HIGH); 
      delay(50); 
    }
  }

  void recalibrate() {
    unsigned long start = millis();
    boolean flash = false;
    while (!calibrated) {
      calibrate();
      if (millis() > start + 50) {   
        digitalWrite(RED_LED, flash ? HIGH : LOW);
        digitalWrite(GREEN_LED, flash ? HIGH : LOW);
        flash = !flash; 
        start = millis();
      }
    }
    digitalWrite(RED_LED, LOW);
  }

  void describe(File *file) {
    file->println("START SENSOR CALIBRATION");
    file->print("accXcal,");
    file->println(accXcal, 12);
    file->print("accYcal,");
    file->println(accYcal, 12);
    file->print("accZcal,");
    file->println(accZcal, 12);
    file->print("gyroXcal,");
    file->println(gyroXcal, 12);
    file->print("gyroYcal,");
    file->println(gyroYcal, 12);
    file->print("gyroZcal,");
    file->println(gyroZcal, 12);
    file->print("altitudeCal,");
    file->println(altitudeCal, 12);
    file->print("temperature,");
    file->println(temperature, 12);
    file->println("END SENSOR CALIBRATION");
    file->flush();
  }

  private:
  void resetCalibrationMetrics() {
    accXcal = 0.0;
    accYcal = 0.0;
    accZcal = 0.0;
    gyroXcal = 0.0;
    gyroYcal = 0.0;
    gyroZcal = 0.0;
    altitudeCal = 0.0;
    calibrationCount = 0;
    calibrated = false;
    // flash in a recognizable pattern
    for (int i = 0; i < 10; i++) {
      digitalWrite(GREEN_LED, HIGH);
      delay(100);
      digitalWrite(GREEN_LED, LOW);
      delay(300);
    }
  }
  
  void calibrate() {
    if (!calibrated) {
      //trap prior accelerometer values
      float ax = accX;
      float ay = accY;
      float az = accZ;
      //update to current values
      update();
      //compare prior to current values
      if (abs(accX) < ACCEL_JITTER && abs(accY) < ACCEL_JITTER && abs(accZ - G) < ACCEL_JITTER) {
        if (calibrationCount >= MIN_CALIBRATION_COUNT) {
          accXcal /= (float) calibrationCount;
          accYcal /= (float) calibrationCount;
          accZcal /= (float) calibrationCount;
          gyroXcal /= (float) calibrationCount;
          gyroYcal /= (float) calibrationCount;
          gyroZcal /= (float) calibrationCount;
          altitudeCal /= (float) calibrationCount;      
          calibrated = true;
        } else {
          accXcal += accX;
          accYcal += accY;
          accZcal += accZ - G;
          gyroXcal += gyroX;
          gyroYcal += gyroY;
          gyroZcal += gyroZ;
          altitudeCal += altitude;
          calibrationCount++;
        }
      } else {    
        resetCalibrationMetrics();
      }
    }
  }
  
  public:
  boolean isCalibrated() {return calibrated;}
  unsigned long getTimestamp() {return timestamp;}
  float getTimespan() {return timespan;}
  float getTemperature() {return temperature;}
  float getAccX() {return accX - accXcal;}
  float getAccY() {return accY - accYcal;}
  float getAccZ() {return accZ - accZcal;}
  float getGyroX() {return gyroX - gyroXcal;}
  float getGyroY() {return gyroY - gyroYcal;}
  float getGyroZ() {return gyroZ - gyroZcal;}
  float getAltitude() {return altitude - altitudeCal;}
};

class LowPassFilter {

  private:
  
  float *accXbuffer;
  float *accYbuffer;
  float *accZbuffer;
  float *altBuffer;
  int i;
  int size;

  public:
  
  LowPassFilter(int buffersize) {
    size = buffersize;
    accXbuffer = new float[size];
    accYbuffer = new float[size];
    accZbuffer = new float[size];
    altBuffer = new float[size];
    for (int n = 0; n < size; n++) {
      accXbuffer[n] = 0.0;
      accYbuffer[n] = 0.0;
      accZbuffer[n] = 0.0;
      altBuffer[n] = 0.0;
    }
    i = 0;
  }
  
  void update(SensorReadings *sensorReadings) {
    accXbuffer[i] = sensorReadings->getAccX();
    accYbuffer[i] = sensorReadings->getAccY();
    accZbuffer[i] = sensorReadings->getAccZ();
    altBuffer[i] = sensorReadings->getAltitude();
    i = ++i % size;
  }
  
  float getAccX() {return getAverageValue(accXbuffer);}
  float getAccY() {return getAverageValue(accYbuffer);}
  float getAccZ() {return getAverageValue(accZbuffer);}
  float getAltitude() {return getAverageValue(altBuffer);}
  
  private:
  float getAverageValue(float *vals) {
    float sum = 0.0;
    for (int n = 0; n < size; n++) sum += vals[n];
    return sum / (float) size;
  }
};

class GyroIntegrator {
  
  private:
  
  float ix;
  float iy;
  float iz;
  
  public:
  
  GyroIntegrator() {
    ix = 0.0;
    iy = 0.0;
    iz = 0.0;
  }

  void update(SensorReadings *s) {
    ix += s->getGyroX() * RADIANS_TO_DEGREES * s->getTimespan();
    iy += s->getGyroY() * RADIANS_TO_DEGREES * s->getTimespan();
    iz += s->getGyroZ() * RADIANS_TO_DEGREES * s->getTimespan();
    //TODO: add correction based upon accelerometer, if in bounds.
  }

  double getRoll() {
    return iy;
  }

  double getPitch() {
    return ix;
  }

  double getYaw() {
    return iz;
  }
};

class Elevator {

  private:
  
  const float maxPitch =  10.0;
  const float minPitch = -10.0;
  const float maxDelta =   2.0;
  const float minDelta =  -2.0;
  
  float pitch;

  public:
  Elevator() {
    pitch = 0.0;
  }

  void setPitch(float p) {
    p = p > maxPitch ? maxPitch : p;
    p = p < minPitch ? minPitch : p;
    float delta = p - pitch;
    delta = delta > maxDelta ? maxDelta : delta;
    delta = delta < minDelta ? minDelta : delta;
    pitch += delta;
    //here - scale to servo width (milliseconds), set servo output pin
  }
};

class Ailerons {

  private:
  
  const float maxPitch =  10.0;
  const float minPitch = -10.0;
  const float maxDelta =   2.0;
  const float minDelta =  -2.0;
  
  float roll;

  public:
  Ailerons() {
    roll = 0.0;
  }

  void setRoll(float r) {
    r = r > maxPitch ? maxPitch : r;
    r = r < minPitch ? minPitch : r;
    float delta = r - roll;
    delta = delta > maxDelta ? maxDelta : delta;
    delta = delta < minDelta ? minDelta : delta;
    roll += delta;
    //here - scale to servo width (milliseconds), set servo output pin
  }
};

class ESC {
  private:
  int throttleMs;
  int rangeMs;
  boolean armed;
  Servo esc;
  
  public:
  ESC() {
    esc.attach(ESC_PIN);
    throttleMs = SERVO_MIN_MICROSECONDS;
    rangeMs = SERVO_MAX_MICROSECONDS - SERVO_MIN_MICROSECONDS;
    armed = false;
  }

  boolean isArmed() {return armed;}

  void arm() {
    //arm esc -- mimics stick movements from radio control
    //go to min, wait, then max, wait, then min
    esc.writeMicroseconds(SERVO_MIN_MICROSECONDS);
    delay(2000);
    esc.writeMicroseconds(SERVO_MAX_MICROSECONDS);
    delay(2000);
    esc.writeMicroseconds(SERVO_MIN_MICROSECONDS);
    delay(2000);
    armed = true;
  }

  void setThrottle(float v) {
    v = v > 10.0 ? 10.0 : v;
    v = v <  0.0 ?  0.0 : v;
    throttleMs = SERVO_MIN_MICROSECONDS + (v/10.0) * rangeMs;
  }
};

// data logging file
File datalog;

// Low pass filter for accelerometer data
SensorReadings *readings;
LowPassFilter *lpf;
GyroIntegrator *gyroIntegrator;
ESC *esc;

//Control surfaces
Elevator *elevator;
Ailerons *ailerons;

unsigned long start_time;
unsigned long blink_time;
unsigned int flight_mode;

boolean blink;

void setup() { 
  initializeLEDs();
  initializeSdCard();
  datalog = getNextAvailableFileHandle();
  readings = new SensorReadings();
  lpf = new LowPassFilter(ACC_FILTER_BUFFER_SIZE);
  gyroIntegrator = new GyroIntegrator();
  esc = new ESC();
  ailerons = new Ailerons();
  elevator = new Elevator();
  writeFileHeader();
  start_time = millis();
  blink_time = start_time;
  flight_mode = BL_PREFLIGHT_CALIBRATE;
}

void loop() {
  
  // read sensors and update related data structures
  readings->update();
  lpf->update(readings);
  gyroIntegrator->update(readings);

  if (flight_mode & FM_TERMINATE) {
    esc->setThrottle(0.0);
    ailerons->setRoll(0.0);
    elevator->setPitch(10.0);
  } else if (flight_mode & BL_PREFLIGHT_CALIBRATE) {
    readings->recalibrate();
    readings->describe(&datalog);
    flight_mode = readings->isCalibrated() ? BL_PREFLIGHT_ARM : FM_TERMINATE;
  } else if (flight_mode & BL_PREFLIGHT_ARM) {
    esc->arm();
    if (esc->isArmed()) {
      waitForPitch();
      flight_mode = FM_TAKEOFF;
    } else {
      flight_mode = FM_TERMINATE;
    }
  } else if (flight_mode & FM_TAKEOFF) {
    
  } else if (flight_mode & FM_LAND_IMMEDIATELY) {
    landImmediately();
  } else if (flight_mode & FM_MAINTAIN_ALTITUDE) {
  } else if (flight_mode & FM_RECALIBRATE_AHRS) {
    recalibrateAHRS();
  } else if (flight_mode & FM_SEEK_WAYPOINT) {
    seekWaypoint();
  } else if (flight_mode & FM_LOITER) {
    loiter();
  } else {
    flight_mode = FM_LAND_IMMEDIATELY;
  }

  //record sensor readings (should be last item in loop(),
  //because writing can be slow, and any changes to throttle
  //or control surfaces should not be delayed.
  writeToFile();

  // blink green LED & flush data (so that data is always
  // available regardless of how or when flight ends).
  if (millis() > blink_time + 1500) {
    digitalWrite(GREEN_LED, blink ? LOW : HIGH);
    blink = !blink;
    blink_time = millis();
    datalog.flush();
  }
}

void landImmediately() {
  // here
}

void recalibrateAHRS() {
  // here
}

void loiter() {
  // here
}

void waitForPitch() {
  // here
}

void seekWaypoint() {
  // here
}

void initializeLEDs() {
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
}

void initializeSdCard() {
  if (!SD.begin(CHIP_SELECT)) {
    readings->terminalError();
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

void writeFileHeader() {
  datalog.println("timestamp,timespan(s),altitude,temperature,accX,accY,accZ,gyroX,gyroY,gyroZ,pitch,roll,yaw");
}

void writeToFile() {
  datalog.print(readings->getTimestamp());
  datalog.print(",");
  datalog.print(readings->getTimespan(), 12);
  datalog.print(",");
  datalog.print(lpf->getAltitude());
  datalog.print(",");
  datalog.print(readings->getTemperature());
  datalog.print(",");  
  datalog.print(lpf->getAccX(), 6);
  datalog.print(",");
  datalog.print(lpf->getAccY(), 6);
  datalog.print(",");
  datalog.print(lpf->getAccZ(), 6);
  datalog.print(",");
  datalog.print(readings->getGyroX(), 12);
  datalog.print(",");
  datalog.print(readings->getGyroY(), 12);
  datalog.print(",");
  datalog.print(readings->getGyroZ(), 12);
  datalog.print(",");
  datalog.print(gyroIntegrator->getPitch(), 6);
  datalog.print(",");
  datalog.print(gyroIntegrator->getRoll(), 6);
  datalog.print(",");
  datalog.println(gyroIntegrator->getYaw(), 6);
}

