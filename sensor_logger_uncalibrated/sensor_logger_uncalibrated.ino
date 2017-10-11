void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Sensor data logger -- uncalibrated");
  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function below
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}

// Executes when interupt is called
SIGNAL(TIMER0_COMPA_vect) {
  Serial.println(millis());
}

void loop() {
  // put your main code here, to run repeatedly:

}
