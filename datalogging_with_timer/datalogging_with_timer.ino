#include <Arduino.h>
#include "Adafruit_ZeroTimer.h"

// timer tester
Adafruit_ZeroTimer zt3 = Adafruit_ZeroTimer(3);

uint16_t prior;

// the timer 3 callbacks
void Timer3Callback0(struct tc_module *const module_inst)
{
  uint16_t m = millis();
  Serial.println(m - prior);
  prior = m;
}

void setup() {
  Serial.begin(9600);
  Serial.println("Timer callback tester");
  
  zt3.configure(TC_CLOCK_PRESCALER_DIV1024, // prescaler
                TC_COUNTER_SIZE_16BIT,   // bit width of timer/counter
                TC_WAVE_GENERATION_NORMAL_PWM // frequency or PWM mode 
                );

  zt3.setCompare(0, 0xFFFF/48); 
  zt3.setCallback(true, TC_CALLBACK_CC_CHANNEL0, Timer3Callback0);
  zt3.enable(true);  
}

void loop() {

}
