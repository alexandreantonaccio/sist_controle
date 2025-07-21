const int trigPin = 9;
const int echoPin = 10;

float duration; // variable to store pulse duration
float distanceCM; // variable to store distance in CM
float distanceIN; // variable to store distance in IN

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // start with a clean signal
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // send trigger signal
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // return pulse duration in microseconds
  // if set to HIGH, pulseIn() waits for the pin to go from LOW to HIGH
  // stops timing when pin goes back LOW
  duration = pulseIn(echoPin, HIGH);
  // convert m/s to in/microsec
  // 343 m/s = .034 cm/microseconds
  distanceCM = (duration * 0.034) / 2;
  // convert to inches, 1in = 2.54cm
  distanceIN = distanceCM / 2.54;
  // print distance to Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distanceCM);
  Serial.print(" cm | ");
  Serial.print(distanceIN);
  Serial.println(" in");
  delay(100);
}