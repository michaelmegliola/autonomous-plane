void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
    while (!Serial) {
    delay(1);
  }
  Serial.println("hello world");
  pinMode(8, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (true) {
    digitalWrite(8, LOW);
    delay (500);
    digitalWrite(8, HIGH);
    delay (500);
    Serial.println("help");
  }
}
