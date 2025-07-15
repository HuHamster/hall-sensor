/*
Code by: HuHamster
www.youtube.com/@HuHamster
*/

#include <Wire.h>           // Required for I2C communication with the OLED display
#include <Adafruit_GFX.h>   // Core graphics library for Adafruit displays
#include <Adafruit_SSD1306.h> // Library for controlling SSD1306 OLED displays

// OLED display dimensions. These are standard values for 0.96" I2C OLEDs.
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// OLED display reset pin. -1 means no dedicated reset pin is used.
#define OLED_RESET -1

// Initialize the OLED display.
// 0x3C is the standard I2C address for many 128x64 OLEDs.
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- IMPORTANT SETTINGS YOU CAN CHANGE IN THE CODE ---

// 1. Wheel/object diameter in meters.
//    Change this value to the actual diameter of your wheel or rotating object.
const float WHEEL_DIAMETER_METERS = 0.6755; // <--- CHANGE THIS VALUE!

// 2. Display mode for speed on the OLED screen.
//    Choose your desired mode by setting the corresponding number:
//    0 = RPM (revolutions per minute)
//    1 = KM/H (kilometers per hour)
//    2 = M/S (meters per second)
const int DISPLAY_MODE = 1; // <--- CHANGE THIS TO 0, 1, OR 2!

// 3. Display update interval in milliseconds.
//    This interval controls how often the display refreshes.
const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 200; // Update the display every 200 ms

// 4. Maximum time to wait for a new pulse to calculate "instantaneous" speed (in ms).
//    If a new pulse doesn't arrive within this time, the speed is considered zero.
//    For example, if a wheel takes 5 seconds for one revolution, this would be 5000 ms.
const unsigned long MAX_TIME_FOR_ONE_ROTATION_MS = 10000; // 10 seconds (for very slow rotations)

// --- END OF IMPORTANT SETTINGS ---

// Automatically calculate the circumference using the formula: Circumference = PI * Diameter.
const float CIRCUMFERENCE_METERS = PI * WHEEL_DIAMETER_METERS;

// The pin the Hall sensor's signal output is connected to.
// For Arduino Uno, this is D2 or D3.
const int HALL_SENSOR_PIN = 2; // Connect the Hall sensor to digital pin D2!

// Variables for counting rotations and time
volatile unsigned long rotationCount = 0;
volatile unsigned long lastHallTriggerTime = 0;
volatile unsigned long previousHallTriggerTime = 0;
volatile float instantaneousSpeedMs = 0.0;
const unsigned long MIN_TIME_BETWEEN_TRIGGERS_MS = 50;

unsigned long lastDisplayUpdateTime = 0;

// The function called every time the Hall sensor is triggered (an interrupt).
// Use 'void countRotation()' for Arduino Uno, without the IRAM_ATTR attribute.
void countRotation() {
  unsigned long currentTime = millis();

  if (currentTime - lastHallTriggerTime > MIN_TIME_BETWEEN_TRIGGERS_MS) {
    previousHallTriggerTime = lastHallTriggerTime;
    lastHallTriggerTime = currentTime;
    
    rotationCount++;

    if (previousHallTriggerTime != 0) {
        unsigned long timeDiff = lastHallTriggerTime - previousHallTriggerTime;
        if (timeDiff > 0) {
            instantaneousSpeedMs = CIRCUMFERENCE_METERS / (timeDiff / 1000.0);
        } else {
            instantaneousSpeedMs = 0.0;
        }
    } else {
        instantaneousSpeedMs = 0.0;
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("Arduino initialized."));
  Serial.print(F("Wheel diameter: ")); Serial.print(WHEEL_DIAMETER_METERS, 4); Serial.println(F(" m"));
  Serial.print(F("Circumference: ")); Serial.print(CIRCUMFERENCE_METERS, 4); Serial.println(F(" m"));
  Serial.print(F("Display update interval: ")); Serial.print(DISPLAY_UPDATE_INTERVAL_MS); Serial.println(F(" ms"));

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Error: OLED display not found or not initialized!"));
    for (;;);
  }
  display.display();
  delay(2000);
  display.clearDisplay();

  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), countRotation, FALLING);

  display.setTextColor(SSD1306_WHITE);
}

void loop() {
  if (millis() - lastDisplayUpdateTime >= DISPLAY_UPDATE_INTERVAL_MS) {
    lastDisplayUpdateTime = millis();

    float current_rpm = 0.0;
    float current_kmh = 0.0;
    float current_ms = 0.0;

    noInterrupts();
    unsigned long timeSinceLastTrigger = millis() - lastHallTriggerTime;
    float temp_instantaneousSpeedMs = instantaneousSpeedMs;
    interrupts();

    if (lastHallTriggerTime == 0 || timeSinceLastTrigger > MAX_TIME_FOR_ONE_ROTATION_MS) {
      current_rpm = 0.0;
      current_kmh = 0.0;
      current_ms = 0.0;
    } else {
      current_ms = temp_instantaneousSpeedMs;
      current_kmh = current_ms * 3.6;
      current_rpm = (current_ms * 60.0) / CIRCUMFERENCE_METERS;
    }

    display.clearDisplay();
    display.setCursor(0, 0);

    switch (DISPLAY_MODE) {
      case 0:
        display.setTextSize(1);
        display.print(F("RPM:"));
        display.setTextSize(2);
        display.println(current_rpm, 2);
        break;
      case 1:
        display.setTextSize(1);
        display.print(F("KM/H:"));
        display.setTextSize(2);
        display.println(current_kmh, 2);
        break;
      case 2:
        display.setTextSize(1);
        display.print(F("M/S:"));
        display.setTextSize(2);
        display.println(current_ms, 2);
        break;
      default:
        display.setTextSize(1);
        display.print(F("RPM:"));
        display.setTextSize(2);
        display.println(current_rpm, 2);
        break;
    }

    display.setTextSize(1);
    display.setCursor(0, 48);
    display.print(F("D:"));
    display.print(WHEEL_DIAMETER_METERS, 3);
    display.print(F("m C:"));
    display.print(CIRCUMFERENCE_METERS, 3);
    display.print(F("m"));

    display.display();
  }
}