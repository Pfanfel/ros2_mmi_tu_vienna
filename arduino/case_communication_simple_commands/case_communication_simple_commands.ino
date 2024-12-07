// Defining variables and the GPIO pins on Arduino
int redPin = 5;
int greenPin = 6;
int bluePin = 7;

void setup()
{
  Serial.begin(9600); // Match the baud rate with the Python script
  pinMode(LED_BUILTIN, OUTPUT);

  // Defining the pins as OUTPUT
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

void loop()
{
  // Check for incoming commands from ROS2
  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    Serial.print("Received command: ");
    Serial.println(command);

    // Parse and handle the command
    handleCommand(command);
  }

  delay(1000); // Adjust the frequency of sending data
}

// Function to handle the command
void handleCommand(String command)
{
  int led_id, r_value, g_value, b_value;
  if (sscanf(command.c_str(), "%d,%d,%d,%d", &led_id, &r_value, &g_value, &b_value) == 4)
  {
    Serial.print("Parsed command - LED ID: ");
    Serial.print(led_id);
    Serial.print(", R: ");
    Serial.print(r_value);
    Serial.print(", G: ");
    Serial.print(g_value);
    Serial.print(", B: ");
    Serial.println(b_value);

    // Handle the command based on led_id
    if (led_id == 1)
    {
      setColor(r_value, g_value, b_value);
      Serial.println("RGB LED color set");
    }
    // Add more handling for other LED IDs if needed
  }
  else
  {
    Serial.println("Invalid command format");
  }
}

// Function to set the RGB LED color
void setColor(int redValue, int greenValue, int blueValue)
{
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}