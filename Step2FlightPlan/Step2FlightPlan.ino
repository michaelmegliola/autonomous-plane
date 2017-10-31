
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <SteamEngine_AHRS.h>

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


/*  =============================================================================
 *  Gyro Integrator
 *  
 *  Maitains a running total of gyro readings... gyros measure angular velocity,
 *  not position, so actual deflection (pitch, roll, or yaw) equals the sum of
 *  (a) angular velocity multiplied by (b) time.
 */
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

  void update(SteamEngineAHRS *s) {
//    ix += s->getGyroX() * RADIANS_TO_DEGREES * s->getTimespan();
//    iy += s->getGyroY() * RADIANS_TO_DEGREES * s->getTimespan();
//    iz += s->getGyroZ() * RADIANS_TO_DEGREES * s->getTimespan();
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

/*  =============================================================================
 *  Elevator
 *  
 *  Represents the elevator control surface. Includes limits to prevent physical
 *  damage or over-cycling of servo.
 */
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

/*  =============================================================================
 *  Ailerons
 *  
 *  Represents the aileron control surfaces. Includes limits to prevent physical
 *  damage or over-cycling of servo. Note that ailerons are physically linked,
 *  so raising one always lowers the other.
 */
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

/*  =============================================================================
 *  Electronic Speed Control
 *  
 *  Represents the speed control device that governs the electric motor and
 *  propeller. Includes limits to keep throttle setting in bounds (note: the
 *  lower bound, expressed in microseconds, is typically 1000, not zero).
 *  
 *  CAUTION: Must be armed before each use. ESC units typically disarm when
 *  power is removed, but may remain armed in unit remains powered up.
 */
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
    esc.writeMicroseconds(throttleMs);
  }
};

/*  =============================================================================
 *  Main flight control program starts here.
 */
 
// data logging file
File datalog;

// sensors
Adafruit_BMP280 bmp;
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

SteamEngineAHRS *readings;
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
  readings = new SteamEngineAHRS(&accelmag, &gyro, &bmp, GREEN_LED);
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
  gyroIntegrator->update(readings);

  if (flight_mode & FM_TERMINATE) {
    esc->setThrottle(0.0);
    ailerons->setRoll(0.0);
    elevator->setPitch(10.0);
  } else if (flight_mode & BL_PREFLIGHT_CALIBRATE) {
    readings->recalibrate();
    //readings->describe(&datalog);
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
    terminalError();
  }
}

void terminalError() {
  while(true) {
    //code here
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
//  datalog.print(readings->getTimestamp());
//  datalog.print(",");
//  datalog.print(readings->getTimespan(), 12);
//  datalog.print(",");
//  datalog.print(readings->getAltitude());
//  datalog.print(",");
//  datalog.print(readings->getTemperature());
//  datalog.print(",");  
//  datalog.print(readings->getAccX(), 6);
//  datalog.print(",");
//  datalog.print(readings->getAccY(), 6);
//  datalog.print(",");
//  datalog.print(readings->getAccZ(), 6);
//  datalog.print(",");
//  datalog.print(readings->getGyroX(), 12);
//  datalog.print(",");
//  datalog.print(readings->getGyroY(), 12);
//  datalog.print(",");
//  datalog.print(readings->getGyroZ(), 12);
//  datalog.print(",");
//  datalog.print(gyroIntegrator->getPitch(), 6);
//  datalog.print(",");
//  datalog.print(gyroIntegrator->getRoll(), 6);
//  datalog.print(",");
//  datalog.println(gyroIntegrator->getYaw(), 6);
}

