/*
Code by: HuHamster
www.youtube.com/@HuHamster
*/

// Define pins for the LED and sensor.
const int ledPin = 6;     // LED pin.
const int sensorPin = 2;  // Digital sensor pin.

void setup() {
  // Configure pin modes when Arduino starts.
  pinMode(ledPin, OUTPUT);   // ledPin as OUTPUT to control the LED.
  pinMode(sensorPin, INPUT); // sensorPin as INPUT to read data from the sensor.
}

void loop() {
  // Infinite loop: read the sensor and control the LED.

  // Read the sensor state and invert it (!) for the LED.
  // If the sensor is HIGH, the LED will be LOW (off), and vice-versa.
  digitalWrite(ledPin, !digitalRead(sensorPin));

  // Delay for 100 milliseconds for stability.
  delay(100);
}