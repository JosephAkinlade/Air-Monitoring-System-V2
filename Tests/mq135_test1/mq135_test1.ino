int R0 = 176;
int R2 = 1000;
float RS;
float PPM_CO;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // read the input on analog pin 2:
  int sensorValue = analogRead(A2);
  //Convert to voltage:
  float volts = sensorValue * 5;
  volts = volts /1023;
  //Calculate RS
  RS = R2 * (1 - volts);
  RS = RS/volts;
  //Calculate CO PPM
  PPM_CO = 
  Serial.println(PPM_CO);
  delay(500);
}
