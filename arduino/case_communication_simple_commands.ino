void setup() {
  Serial.begin(9600); // Match the baud rate with the Python script
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Send data to ROS2
  Serial.println("Hello from Arduino");

  // Check for incoming commands from ROS2
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    Serial.print("Received command: ");
    Serial.println(command);

    // Handle commands using switch-case
    switch (parseCommand(command)) {
      case 1: // LED1_ON
        blinkPattern1();
        Serial.println("Confirmation: LED1_ON pattern executed");
        break;
      case 2: // LED1_OFF
        blinkPattern2();
        Serial.println("Confirmation: LED1_OFF pattern executed");
        break;
      case 3: // LED2_ON
        blinkPattern3();
        Serial.println("Confirmation: LED2_ON pattern executed");
        break;
      case 4: // LED2_OFF
        blinkPattern4();
        Serial.println("Confirmation: LED2_OFF pattern executed");
        break;
      default: // Unknown command
        Serial.println("Unknown command");
        break;
    }
  }

  delay(1000); // Adjust the frequency of sending data
}

// Helper function to parse commands
int parseCommand(String command) {
  if (command == "LED1_ON") return 1;
  if (command == "LED1_OFF") return 2;
  if (command == "LED2_ON") return 3;
  if (command == "LED2_OFF") return 4;
  return 0; // Unknown command
}

// Blinking patterns
void blinkPattern1() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

void blinkPattern2() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(300);
    digitalWrite(LED_BUILTIN, LOW);
    delay(300);
  }
}

void blinkPattern3() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}

void blinkPattern4() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
  }
}
