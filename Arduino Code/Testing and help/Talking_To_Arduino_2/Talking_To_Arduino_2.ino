void setup() {
  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  // Send a message
  Serial.println("Hello from Uno!");
  delay(1000); // Delay between messages
}
