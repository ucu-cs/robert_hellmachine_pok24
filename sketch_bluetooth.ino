
void setup() {
  pinMode(LED, OUTPUT);
}

void loop() {
  delay(1000);
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
}