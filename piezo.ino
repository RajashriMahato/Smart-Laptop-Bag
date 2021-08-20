/* Knock Sensor
 * Program using a Piezo element as if it was a knock sensor.
 */

int ledPin = 13;
int knockSensor = A7;               
byte val = 0;
int statePin = LOW;
int THRESHOLD = 100;

void setup() {
 pinMode(ledPin, OUTPUT);
 pinMode(knockSensor, INPUT);
 Serial.begin(9600);
}

void loop() {
  val = analogRead(knockSensor);     
  /*if (val >= THRESHOLD) {
    statePin = !statePin;
    digitalWrite(ledPin, statePin);
    Serial.println("Knock!");
  }*/
  Serial.println(val);
  delay(100);  // we have to make a delay to avoid overloading the serial port
}
