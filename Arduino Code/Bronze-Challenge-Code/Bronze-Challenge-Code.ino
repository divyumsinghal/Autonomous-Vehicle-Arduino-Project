// Some necessary stuff

// URLs for different car commands
#define startURL 'Z'
#define stopURL 'X'
#define displayHeartURL 'H'
#define displaySmileyURL 'S'
#define displayW5URL 'W'

// Include the library for handling an LED matrix
#include "Arduino_LED_Matrix.h"
// Include the library for WiFi communication using the NINA module
#include <WiFiS3.h>

// Variables

// Pin Definitions
const int LeftIRSensorInput = A1;               // Analog pin for the left infrared sensor
const int RightIRSensorInput = A0;              // Analog pin for the right infrared sensor
const int RightMotorPWM = 10;                   // PWM pin for controlling the right motor speed
const int LeftMotorPWM = 9;                     // PWM pin for controlling the left motor speed
const int RightMotorSwitch1 = 11;               // Motor control pin 1 for the right motor
const int RightMotorSwitch2 = 12;               // Motor control pin 2 for the right motor (changed to 9, used to be 13)
const int LeftMotorSwitch3 = 8;                 // Motor control pin 3 for the left motor
const int LeftMotorSwitch4 = 7;                 // Motor control pin 4 for the left motor
const int UltrasonicTrigger = 5;                // Digital pin for triggering the ultrasonic sensor
const int UltrasonicEchoDetector = 4;           // Digital pin for detecting the echo from the ultrasonic sensor
const int LED_PIN = 13;                         // Digital pin for the LED
int RightMotorSwitchActive = RightMotorSwitch1; // Active Switch for right motor
int LeftMotorSwitchActive = LeftMotorSwitch3;   // Active Switch for Left motor

// Defining encoders:
const int RightEncoder = 3;        // Digital pin for the right motor encoder
const int LeftEncoder = 2;         // Digital pin for the left motor encoder
unsigned long leftPulseCount = 0;  // Pulse count for the left motor encoder
unsigned long rightPulseCount = 0; // Pulse count for the right motor encoder

// Constants
const float RightWheelCoefficient = 1;    // Coefficient for adjusting Right wheel speed
const float LeftWheelCoefficient = 0.92;  // Coefficient for adjusting left wheel speed
const float MinSpeed = 0;                 // Minimum speed scale for the car
const float MaxSpeed = 100;               // Maximum speed scale for the car
const int PWMMin = 0;                     // Minimum PWM value
const int PWMMax = 125;                   // Maximum PWM value (capping it at 125 instead of 225)
const int TurnSpeedOuter = 100;           // Turning speed for outer wheel
const int TurnSpeedInner = 30;            // Turning speed for inner wheel
const int EncoderPulsesPerRevolution = 4; // Encoder generates 8 pulses per revolution -> 4 rising
const int CriticalObjectDistance = 25;    // Critical distance for detecting obstacles

const int Overtime = 100;                     // Return this when sonar takes too long
const float SPEED_OF_SOUND_CM_PER_MS = 0.017; // Conversion factor for microseconds to distance
const float radiusOfWheel = 3.5;              // radius of wheel in cm

// more variables
float nearestObstacleDistance = 100;                                                                                       // Distance to the nearest obstacle from ultrasonic sensor
bool obstacleTooClose = false;                                                                                             // Flag indicating if an obstacle is too close
float carSpeed = MaxSpeed;                                                                                                 // Current speed of the car
unsigned long loopCounter = 0;                                                                                             // Counter for obstacle tracking frequency
bool StopTheCarThroughGUI = false;                                                                                         // Control if you want the car to move
String message;                                                                                                            // Message to send to the client
float distanceTravelledByTheCar = (leftPulseCount + rightPulseCount) * 3.142 * radiusOfWheel / EncoderPulsesPerRevolution; // How far have the wheels spun (in cm)

// Wifi

// Wifi Details
char ssid[] = "w5shouldget100";
char pass[] = "bestgroupfr123";

// Declare an instance of the WiFiServer class named 'server'
WiFiServer server(5200);
// Declare a client (needs to be available globally for 2 way communication across)
WiFiClient ProcessingClient;

// WiFi Setup

// Connection Setup
void connectionSetup()
{
  Serial.println("Inside connection setup");

  // Initiate a connection to the WiFi network using the provided SSID and password
  WiFi.beginAP(ssid, pass);

  delay(5000);

  // Obtain the local IP address assigned to the Arduino on the WiFi network
  IPAddress ip = WiFi.localIP();

  // Print a label indicating the following content on the Serial Monitor
  Serial.print("IP Address:");

  // Print the obtained IP address to the Serial Monitor
  Serial.println(ip);

  // Start the WiFiServer to listen for incoming connections on the specified port
  server.begin();
}

// Check client connection
void checkServer()
{

  // Serial.print('_');

  // Send a "Hello Client" message to the connected client
  //("Hello Client");

  // Serial.print("Client is connected!");
  char data = ProcessingClient.read();

  switch (data)
  {
  case startURL:
    Serial.println(data);
    StopTheCarThroughGUI = false;
    moveForwardatSpeed(carSpeed);
    // Serial.println(data);
    break;

  case stopURL:
    // Serial.println(data);
    StopTheCarThroughGUI = true;
    stopCar();
    break;

  case displayHeartURL:
    // Serial.println(data);
    displayHeart();
    break;

  case displaySmileyURL:
    // Serial.println(data);
    displaySmiley();
    break;

  case displayW5URL:
    // Serial.println(data);
    displayW5();
    break;

  default:
    // Handle unknown command
    // Serial.print(data);
    break;
  }
}

// Matrix Handling
ArduinoLEDMatrix matrix;

// Predefined patterns for the LED matrix

// Smiley pattern
const uint32_t happy[3] = {
    0x19819,
    0x80000001,
    0x81f8000};

// Heart pattern
const uint32_t heart[3] = {
    0x3184a444,
    0x44042081,
    0x100a0040};

// W5 pattern
const uint32_t W5[4] = {
    0x82f82882,
    0x892fba1e,
    0xe1ee1c6f,
    66};

// Functions needed for Matrix

// Function to display a smiley pattern on the LED matrix
void displaySmiley()
{
  matrix.loadFrame(happy);

  // delay(500);
  //  Load the predefined smiley pattern onto the LED matrix
  //  Delay is intentionally commented out for potential real-time or non-blocking behavior
}

// Function to display a heart pattern on the LED matrix
void displayHeart()
{
  matrix.loadFrame(heart);

  // delay(500);
  //  Load the predefined heart pattern onto the LED matrix
}

// Function to display a W5 pattern on the LED matrix
void displayW5()
{
  matrix.loadFrame(W5);
  // delay(500);
}

// Functions needed for Movement

// Mapping speed to PWM values
int mapSpeedToPWM(float speed)
{
  int PWMValue = round(map(speed, MinSpeed, MaxSpeed, PWMMin, PWMMax));

  // Serial.println("Speed: ");
  // Serial.println(speed);
  // Serial.println(" | Mapped PWM: ");
  // Serial.println(floor(PWMValue));

  return PWMValue;
}

// Function to move the car forward at a specified speed
void moveForwardatSpeed(float speed)
{
  // Serial.print("Moving forward at ");
  // Serial.println(speed);

  // Adjust right motor PWM based on the specified speed
  analogWrite(RightMotorPWM, mapSpeedToPWM(RightWheelCoefficient * speed));

  // Adjust left motor PWM based on the left wheel coefficient
  analogWrite(LeftMotorPWM, mapSpeedToPWM(LeftWheelCoefficient * speed));

  // Switch on
  digitalWrite(RightMotorSwitchActive, HIGH);
  digitalWrite(LeftMotorSwitchActive, HIGH);
}

// Function to stop the car
void stopCar()
{
  // Serial.println("Stopping car");

  // Stop the right motor by setting its PWM to 0
  analogWrite(RightMotorPWM, mapSpeedToPWM(0));

  // Stop the left motor by setting its PWM to 0
  analogWrite(LeftMotorPWM, mapSpeedToPWM(0));

  digitalWrite(RightMotorSwitchActive, LOW);
  digitalWrite(LeftMotorSwitchActive, LOW);
}

// Function to turn the car to the left
void turnLeft()
{
  // Serial.println("Inside turnLeft - Turning left");

  // Adjust the right motor PWM for a left turn
  analogWrite(RightMotorPWM, mapSpeedToPWM(RightWheelCoefficient * TurnSpeedOuter));

  // Stop the left motor by setting its PWM to 0
  analogWrite(LeftMotorPWM, mapSpeedToPWM(LeftWheelCoefficient * TurnSpeedInner));

  digitalWrite(RightMotorSwitchActive, HIGH);
  digitalWrite(LeftMotorSwitchActive, HIGH);
}

// Function to turn the car to the right
void turnRight()
{
  // Serial.println("Inside turnRight - Turning right");

  // Stop the right motor by setting its PWM to 0
  analogWrite(RightMotorPWM, mapSpeedToPWM(RightWheelCoefficient * TurnSpeedInner));

  // Adjust the left motor PWM for a right turn
  analogWrite(LeftMotorPWM, mapSpeedToPWM(LeftWheelCoefficient * TurnSpeedOuter));

  // Switch on
  digitalWrite(RightMotorSwitchActive, HIGH);
  digitalWrite(LeftMotorSwitchActive, HIGH);
}

// Sensor Functions

// Function to measure the closest obstacle distance using the ultrasonic sensor
float closestObstacleUsingSonar()
{
  // Set trigger pin low to ensure a clean pulse
  digitalWrite(UltrasonicTrigger, LOW);

  // Wait for a brief interval to stabilize the signal
  delayMicroseconds(2);

  // Generate a 10-microsecond pulse to trigger the ultrasonic sensor
  digitalWrite(UltrasonicTrigger, HIGH);

  // Keep the trigger pulse active for a specific duration
  delayMicroseconds(10);

  // Deactivate the trigger pulse
  digitalWrite(UltrasonicTrigger, LOW);

  // Measure the duration of the pulse , only wait for 6000 microseconds
  unsigned long pulseDuration = pulseIn(UltrasonicEchoDetector, HIGH, 6000);

  // Convert the pulse duration to distance using the speed of sound
  // Return the distance or Overtime if no pulse received within 2 seconds

  return (pulseDuration > 0) ? pulseDuration * SPEED_OF_SOUND_CM_PER_MS : Overtime;
}

// Checking the position relative to obstacles using the ultrasonic sensor
void checkPositionRelativeToObject()
{
  nearestObstacleDistance = closestObstacleUsingSonar();

  // Serial.println("Nearest obstacle distance: ");
  // Serial.println(nearestObstacleDistance);

  // Logic for handling obstacle proximity and adjusting car speed

  if (nearestObstacleDistance <= CriticalObjectDistance)
  {

    // If obstacle is too close, stop the car
    obstacleTooClose = true;
    stopCar();

    return;
  }
  else
  {

    obstacleTooClose = false;

    // Serial.println("Inside Object Tracking, changing speed to: " + String(carSpeed));
  }
}

// Keeping the car moving while checking infrared sensors for tracking
void keepMovingCheckingIRSensors()
{
  // Serial.println("got inside keepMovingCheckingIRSensors");

  int irLeftValue = digitalRead(LeftIRSensorInput);
  int irRightValue = digitalRead(RightIRSensorInput);

  // Serial.println(irLeftValue);
  // Serial.println(irRightValue);

  if (irLeftValue == LOW && irRightValue == HIGH)
  {

    // Serial.println("Left sensor off track - turning left");

    // If left sensor is off track, turn left

    turnLeft();
  }
  else if (irLeftValue == HIGH && irRightValue == LOW)
  {

    // Serial.println("Right sensor off track - turning Right");

    // If right sensor is off track, turn Right

    turnRight();
  }
  else
  {

    // Serial.println("Both sensors on track - moving forward");

    // If both sensors are on track, move forward

    moveForwardatSpeed(carSpeed);
  }

  // Serial.println("end of keepMovingCheckingIRSensors");
}

// Initialization function executed once at the start of the program

void setup()
{

  Serial.begin(9600);                     // Initialize Serial communication with a baud rate of 9600
  pinMode(LeftIRSensorInput, INPUT);      // Configure the left infrared sensor pin as an input
  pinMode(RightIRSensorInput, INPUT);     // Configure the right infrared sensor pin as an input
  pinMode(UltrasonicTrigger, OUTPUT);     // Configure the ultrasonic trigger pin as an output
  pinMode(UltrasonicEchoDetector, INPUT); // Configure the ultrasonic echo detector pin as an input
  pinMode(LeftMotorSwitch3, OUTPUT);      // Configure the left motor control pin 3 as an output
  pinMode(LeftMotorSwitch4, OUTPUT);      // Configure the left motor control pin 4 as an output
  pinMode(RightMotorSwitch1, OUTPUT);     // Configure the right motor control pin 1 as an output
  pinMode(RightMotorSwitch2, OUTPUT);     // Configure the right motor control pin 2 as an output

  digitalWrite(RightMotorSwitch1, LOW); // Initially set both motors to be swtiched off
  digitalWrite(RightMotorSwitch2, LOW);
  digitalWrite(LeftMotorSwitch3, LOW);
  digitalWrite(LeftMotorSwitch4, LOW);

  pinMode(LeftMotorPWM, OUTPUT);       // Configure the left motor PWM pin as an output
  pinMode(RightMotorPWM, OUTPUT);      // Configure the right motor PWM pin as an output
  pinMode(LeftEncoder, INPUT_PULLUP);  // Configure the left motor encoder pin with pull-up resistor enabled & logic inversion (a 20 k resistor in parallel for impedence control)
  pinMode(RightEncoder, INPUT_PULLUP); // Configure the right motor encoder pin with pull-up resistor enabled

  matrix.begin(); // Initialize the LED matrix for further use

  // Setup Connection
  // Serial.println("Inside Setup befor connection");

  Serial.println("Setting up!");

  connectionSetup();

  // Serial.println("Inside Setup afer connection");

  // Interrupts

  // Serial.println("1 Inside Setup 2");

  attachInterrupt(
      digitalPinToInterrupt(RightEncoder), []()
      { rightPulseCount++; },
      RISING);

  attachInterrupt(
      digitalPinToInterrupt(LeftEncoder), []()
      { leftPulseCount++; },
      RISING);

  // Serial.println(" 4 Inside Setup 3");

  while (!ProcessingClient.connected())
  {
    // Attempt to accept an incoming client connection on the WiFi server
    ProcessingClient = server.available();

    Serial.print("-");

    delay(100);
  }

  Serial.println("Connected!");
}

// Main execution loop function that runs continuously after setup

void loop()
{
  // Delegate the primary control logic
  // Serial.print('.');

  // Serial.println("starting loop");  // Optional debugging statement
  if (loopCounter % 29 == 0)
  {

    checkServer();
  }
  else if (loopCounter % 32 == 0)
  {

    checkPositionRelativeToObject();
  }
  else if (loopCounter == 500)
  {

    // Execute obstacle tracking logic
    // Serial.print("Got into the 25th loop");

    distanceTravelledByTheCar = (leftPulseCount + rightPulseCount) * 3.142 * radiusOfWheel / EncoderPulsesPerRevolution;

    // message = "Distance travelled: " + String(int(distanceTravelledByTheCar))
    //           + ((nearestObstacleDistance != 100) ? " Object at: " + String(int(nearestObstacleDistance)) + "\n" : " No Object \n");

    // Serial.println(message);
    // ProcessingClient.write(message.c_str(), message.length());

    message = "Distance travelled: " + String(int((leftPulseCount + rightPulseCount) * 3.142 * radiusOfWheel / EncoderPulsesPerRevolution)) + " Left travelled: " + String(int((leftPulseCount) * 3.142 * radiusOfWheel / EncoderPulsesPerRevolution)) + " Right travelled: " + String(int((rightPulseCount) * 3.142 * radiusOfWheel / EncoderPulsesPerRevolution)) + ((nearestObstacleDistance != 100) ? " Object at: " + String(int(nearestObstacleDistance)) + "\n" : " No Object \n");

    Serial.println(message);
    // ProcessingClient.write(message.c_str(), message.length());

    // Reset the loop counter for the next iteration

    loopCounter = 0;
  }

  // Serial.println("Is the obstacleTooClose?");  // Optional debugging statement
  // Serial.println(obstacleTooClose);  // Optional debugging statement

  if (!obstacleTooClose && !StopTheCarThroughGUI)
  {

    // Continue moving and checking infrared sensors if no obstacles are too close
    //  Serial.print("MOVE BRO?");  // Optional debugging statement

    keepMovingCheckingIRSensors();
  }
  else if (!StopTheCarThroughGUI) // obstacleTooClose &&
  {

    delayMicroseconds(100);

    checkPositionRelativeToObject();
  }

  // Increment the loop counter for each iteration
  loopCounter++;

  // Serial.println("end of loop");
}

// End

/*

int main ()
{
  setup ();

  while (true)
  {
    loop();
  }
}

*/
