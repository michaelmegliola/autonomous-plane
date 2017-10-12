/*
 * Step 1. Raw data logging.
 * 
 * 
 * 
 * For the code that sets up the timer and callback function,
 * special thanks to Sheng Chen https://gist.github.com/jdneo
 */
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <SPI.h>
#include <SD.h>

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

#define BMP_SCK 10    // BMP280 pin assignments
#define BMP_MISO 11
#define BMP_MOSI 12 
#define BMP_CS 13

#define GREEN_LED 8   // Onboard LED
#define RED_LED 13    // Onboard LED
#define CHIP_SELECT 4 // SDI (micro SD memory card)

// sensors
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

// data logging file
File datalog;

uint16_t prior_event_time;
uint16_t start_time;
boolean stop_logging;
boolean shut_down_complete;
boolean red_flash;

void setup() {  
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  initializeSdCard();
  initializeSensorArray();
  datalog = getNextAvailableFileHandle();
  stop_logging = false;
  start_time = millis();
  startTimer(100);
}

void loop() {
  stop_logging |= (millis() > start_time + 600000);
  if (stop_logging) {
    datalog.flush();
    datalog.close();
    while (true) {
      digitalWrite(GREEN_LED, HIGH);
      delay(300);
      digitalWrite(GREEN_LED, LOW);
      delay(100);
    }
  }
}

void terminalError() {
  while(true) {
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    delay(100);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH); 
    delay(80); 
  }
}

void initializeSdCard() {
  if (!SD.begin(CHIP_SELECT)) {
    terminalError();
  }
}

void initializeSensorArray() {
  if (!bmp.begin()) {  
    terminalError();
  }
  if(!gyro.begin(GYRO_RANGE_250DPS)) {
    terminalError();
  }
  if(!accelmag.begin(ACCEL_RANGE_4G)) {
    terminalError();
  }
}

String getFileName(int i) {
  String s = "DATA";
  s += i;
  s += ".txt";
  return s;
}

File getNextAvailableFileHandle() {
  int i = 0;
  while (SD.exists(getFileName(i))) {
    i++;
  }
  return SD.open(getFileName(i), FILE_WRITE);
}

void timerCallback() {
  
  if (stop_logging) return;
  
  // get a new sensor event
  uint16_t event_time = millis();
  sensors_event_t event;
  gyro.getEvent(&event);
  accelmag.getEvent(&event);
    
  // write raw sensor readings to data file
  String dataString = "";
  dataString += (event_time - prior_event_time);
  dataString += ",";
  dataString += micros();
  dataString += ",";
  dataString += bmp.readTemperature();
  dataString += ",";
  dataString += bmp.readAltitude(1013.25);
  dataString += ",";
  dataString += event.gyro.x;  
  dataString += ",";
  dataString += event.gyro.y;  
  dataString += ",";
  dataString += event.gyro.z;
  dataString += ",";
  dataString += event.acceleration.x;
  dataString += ",";
  dataString += event.acceleration.y;
  dataString += ",";
  dataString += event.acceleration.z;
  dataString += ",";
  dataString += accelmag.accel_raw.x;
  dataString += ",";
  dataString += accelmag.accel_raw.y;
  dataString += ",";
  dataString += accelmag.accel_raw.z;
 
  if (datalog) {
    datalog.println(dataString); 
  } else {
    terminalError();
  }
  
  prior_event_time = event_time;
}

void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // then invoke the callback function
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    timerCallback();
  }
  if (red_flash) {
    digitalWrite(RED_LED, HIGH);
  } else {
    digitalWrite(RED_LED, LOW);
  }
  red_flash = !red_flash;
}
