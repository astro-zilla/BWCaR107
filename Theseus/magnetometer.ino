void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  float inputvoltage = 5.0;
  int sensor = analogRead(A0);
  float voltage = sensor * inputvoltage / 1023.0;
  Serial.println(sensor);
}
