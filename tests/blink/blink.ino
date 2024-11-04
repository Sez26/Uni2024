const int ledPin = 25; // Built-in LED on Raspberry Pi Pico (GPIO 25)
int blinkInterval = 1000; // Interval for LED blink (ms)
unsigned long previousMillis = 0; // Tracks the last time the LED was toggled

void setup() {
  // Initialize the built-in LED pin as an output
  pinMode(ledPin, OUTPUT);

  // Initialize Serial communication at 115200 baud
  Serial.begin(115200);
  while (!Serial) {
    // Wait for Serial to initialize (only necessary on some boards)
  }
  
  Serial.println("Raspberry Pi Pico Blink and Serial Monitor Test Initialized");
}

void loop() {
  // Check the time to see if it's time to toggle the LED
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= blinkInterval) {
    previousMillis = currentMillis; // Update the last toggle time

    // Toggle the LED state
    digitalWrite(ledPin, !digitalRead(ledPin));

    // Print the LED status to the Serial Monitor
    if (digitalRead(ledPin) == HIGH) {
      Serial.println("LED is ON");
    } else {
      Serial.println("LED is OFF");
    }
  }
}
