#include <Arduino.h>
#line 1 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
// URLs for different car commands
#define startURL 'Z'
#define stopURL 'X'
#define displayHeartURL 'H'
#define displaySmileyURL 'S'
#define displayW5URL 'W'
#define mode1URL 'A'
#define mode2URL 'B'

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
const int RightEncoder = 3;                      // Digital pin for the right motor encoder
const int LeftEncoder = 2;                       // Digital pin for the left motor encoder
volatile unsigned long long leftPulseCount = 0;  // Pulse count for the left motor encoder
volatile unsigned long long rightPulseCount = 0; // Pulse count for the right motor encoder

// Constants
const double RightWheelCoefficient = 1;        // Coefficient for adjusting Right wheel speed
const double LeftWheelCoefficient = 0.92;      // Coefficient for adjusting left wheel speed
const double MinSpeedMmS = 0;                  // Minimum speed scale for the car
const double MaxSpeedMmS = 90;                 // Maximum speed scale for the car
const int PWMMin = 0;                          // Minimum PWM value
const int PWMMax = 140;                        // Maximum PWM value (capping it at 140 instead of 225)
const int TurnSpeedOuterPulseWidth = 130;      // Turning speed for outer wheel
const int TurnSpeedInnerPulseWidth = 50;       // Turning speed for inner wheel
const int EncoderPulsesPerRevolution = 4;      // Encoder generates 8 pulses per revolution -> 4 rising
const int CriticalObjectDistance = 10;         // Critical distance for detecting obstacles
const int ObjectFollowingDistance = 20;        // A slightly larger and safer distance
const int Overtime = 35;                       // Return this when sonar takes too long
const double SPEED_OF_SOUND_CM_PER_MS = 0.017; // Conversion factor for microseconds to distance
const double radiusOfWheelCm = 3.5;            // radius of wheel in cm
const double radiusOfWheelMm = 35;             // radius of wheel in mm
const long micro = 1e6;

// more variables
double nearestObstacleDistance = 100;                                                                                           // Distance to the nearest obstacle from ultrasonic sensor
bool obstacleTooClose = false;                                                                                                  // Flag indicating if an obstacle is too close
double carSpeedAlmostMmS = MaxSpeedMmS;                                                                                         // Current speed of the car
unsigned long loopCounter = 0;                                                                                                  // Counter for obstacle tracking frequency
bool StopTheCar = true;                                                                                                         // Control if you want the car to move
double distanceTravelledByTheCarCm = (leftPulseCount + rightPulseCount) * 3.142 * radiusOfWheelMm / EncoderPulsesPerRevolution; // How far have the wheels spun (in cm)
double targetSpeedMmS = MaxSpeedMmS;                                                                                            // Speed to reach in mode 2

// MODES

// Define an enumeration for modes
enum Modes
{
  MODE_1_Object_Following, // Object Following -> PID
  MODE_2_Speed_Control     // Speed Control -> Slider on GUI
};

// Variable to store the current mode
Modes currentMode = MODE_2_Speed_Control;

// Speed of Buggy

double arcLengthMm = double(radiusOfWheelMm * 45 / 360);

volatile unsigned long long leftTimePrev = micros();
volatile unsigned long long leftTimeCurrent = micros();

volatile unsigned long long rightTimePrev = micros();
volatile unsigned long long rightTimeCurrent = micros();

double leftSpeedMmS = double(arcLengthMm / (leftTimeCurrent - leftTimePrev)); // cm/s
double rightSpeedMmS = double(arcLengthMm / (rightTimeCurrent - rightTimePrev));
double averageSpeedMmS = double((leftSpeedMmS + rightSpeedMmS) / 2);

// PID for Object Following

// Define PID constants
const double Kp_f_1 = 0.1;       // Proportional gain
long const double Ki_f_1 = 5e-9; // Integral gain
const double Kd_f_1 = 400;       // Derivative gain

// Define variables
double error_f_1 = 0;
double previousError_f_1 = 0;
double integral_f_1 = 0;
double differential_f_1 = 0;
double controlSignal_f_1 = 0;
double currentTime_f_1 = micros();
double previousTime_f_1 = micros();
double elapsedTime_f_1 = 0;

// Define function
#line 109 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
double PIDSpeedObjectFollowing();
#line 177 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
double PIDSpeedMaintainSpeed();
#line 239 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
void connectionSetup();
#line 260 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
void checkServer();
#line 336 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
void sendMessageCSV();
#line 345 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
void sendMessage();
#line 376 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
void calculateSpeed();
#line 412 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
void displaySmiley();
#line 422 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
void displayHeart();
#line 431 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
void displayW5();
#line 440 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
int mapSpeedMmSToPWM(double speed);
#line 453 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
void moveForwardatSpeed(double speed);
#line 473 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
void stopCar();
#line 488 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
void turnLeft();
#line 503 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
void turnRight();
#line 521 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
double closestObstacleUsingSonar();
#line 548 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
void checkPositionRelativeToObject();
#line 592 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
void keepMovingCheckingIRSensors();
#line 634 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
void setup();
#line 710 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
void loop();
#line 109 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Silver-Challenge-Code\\Silver-Challenge-Code.ino"
double PIDSpeedObjectFollowing()
{

  // Time
  currentTime_f_1 = micros();
  elapsedTime_f_1 = double(currentTime_f_1 - previousTime_f_1);

  // Calculate error
  error_f_1 = double(nearestObstacleDistance - ObjectFollowingDistance);

  // Update integral
  integral_f_1 += (error_f_1 < 15) ? double(error_f_1 * elapsedTime_f_1) : 0;

  // Update differential
  differential_f_1 = double((error_f_1 - previousError_f_1) / elapsedTime_f_1);

  // Calculate control signal
  controlSignal_f_1 = double(Kp_f_1 * error_f_1 + Ki_f_1 * integral_f_1 + Kd_f_1 * differential_f_1);

  // Update previous error & time
  previousError_f_1 = error_f_1;
  previousTime_f_1 = currentTime_f_1;

  // Calculate next speed
  double nextSpeed_f_1 = carSpeedAlmostMmS + controlSignal_f_1;

  // Ensure the next speed is within bounds
  nextSpeed_f_1 = constrain(nextSpeed_f_1, MinSpeedMmS, MaxSpeedMmS);

  /*
  message = "Distance travelled: " + String(int(distanceTravelledByTheCar))
            + " Speed: " + String(int(carSpeed))
            + ((nearestObstacleDistance != 100) ? " Object at: " + String(int(nearestObstacleDistance)) : " No Object")
            + " Current mode: " + ((currentMode) ? "Mode2" : "Mode1")
            + " error =           " + String(Kp * error)
            + " integral =        " + String(Ki * integral)
            + " differential =    " + String(Kd * differential)
            + " controlSignal =   " + String(controlSignal)
            + " nextSpeed =       " + String(nextSpeed)
            + "                                          " + " \n";


  // Serial.println(message);
  ProcessingClient.write(message.c_str(), message.length());
  */

  Serial.println("F1 PID: " + String(nextSpeed_f_1));

  return nextSpeed_f_1;
}

// PID for Maintain Speed

// Define PID constants
const double Kp_m_2 = 0.1;       // Proportional gain
const long double Ki_m_2 = 5e-9; // Integral gain
const double Kd_m_2 = 400;       // Derivative gain

// Define variables
double error_m_2 = 0;
double previousError_m_2 = 0;
double integral_m_2 = 0;
double differential_m_2 = 0;
double controlSignal_m_2 = 0;
unsigned long long currentTime_m_2 = micros();
unsigned long long previousTime_m_2 = micros();
double elapsedTime_m_2 = 0;

double PIDSpeedMaintainSpeed()
{

  leftSpeedMmS = double(arcLengthMm / (leftTimeCurrent - leftTimePrev)); // mm/s
  rightSpeedMmS = double(arcLengthMm / (rightTimeCurrent - rightTimePrev));
  averageSpeedMmS = double((leftSpeedMmS + rightSpeedMmS) / 2);

  // Time
  currentTime_m_2 = micros();
  elapsedTime_m_2 = double(currentTime_m_2 - previousTime_m_2);

  // Calculate error
  error_m_2 = double(targetSpeedMmS - averageSpeedMmS);

  // Update integral
  integral_m_2 += (error_m_2 < 15) ? double(error_m_2 * elapsedTime_m_2) : 0;

  // Update differential
  differential_m_2 = double((error_m_2 - previousError_m_2) / elapsedTime_m_2);

  // Calculate control signal
  controlSignal_m_2 = double(Kp_m_2 * error_m_2 + Ki_m_2 * integral_m_2 + Kd_m_2 * differential_m_2);

  // Update previous error & time
  previousError_m_2 = error_m_2;
  previousTime_m_2 = currentTime_m_2;

  // Calculate next speed
  double nextSpeed_m_2 = carSpeedAlmostMmS + controlSignal_m_2;

  // Ensure the next speed is within bounds
  nextSpeed_m_2 = constrain(nextSpeed_m_2, MinSpeedMmS, MaxSpeedMmS);

  Serial.println("M2 PID: " + String(nextSpeed_m_2));

  return nextSpeed_m_2;
}


// Wifi

// Wifi Details
char ssid[] = "SKYC9DAB";
char pass[] = "WWSMWVYRVC";

// Declare an instance of the WiFiServer class named 'server'
WiFiServer server(5200);
// Declare a client (needs to be available globally for 2 way communication across)
WiFiClient ProcessingClient;

// Messages
String messageCSV;

// String message;
// byte messageArr[3] = { 0, 0, 0 };

// Data
char data;

// WiFi Setup

// Connection Setup
void connectionSetup()
{
  Serial.println("Inside connection setup");

  // Initiate a connection to the WiFi network using the provided SSID and password
  WiFi.begin(ssid, pass);

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
  data = ProcessingClient.read();

  switch (data)
  {
  case startURL:

    Serial.println(data);
    StopTheCar = false;
    // Serial.println(data);
    break;

  case stopURL:

    Serial.println(data);
    StopTheCar = true;
    stopCar();
    break;

  case mode1URL:
    currentMode = MODE_1_Object_Following;
    integral_f_1 = 0;

    // ProcessingClient.write("NOW Mode 1! \n");

    break;

  case mode2URL:
    // ProcessingClient.write("NOW Mode 2! \n");

    currentMode = MODE_2_Speed_Control;
    integral_m_2 = 0;

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
    // Handle speed Command
    Serial.print(data);

    if (currentMode == MODE_2_Speed_Control)
    {
      int setSpeed = data - '0';
      if (setSpeed > MinSpeedMmS && setSpeed <= MaxSpeedMmS)
      {
        targetSpeedMmS = setSpeed;
        carSpeedAlmostMmS = targetSpeedMmS;
        // Serial.println("Inside Mode 2, Changing speed to: " + String(carSpeed));
      }
    }
    break;
  }
}

void sendMessageCSV()
{

  messageCSV = String(int(distanceTravelledByTheCarCm)) + "," + String(int(averageSpeedMmS)) + "," + String(int(nearestObstacleDistance)) + " \n ";

  // ProcessingClient.write(messageCSV.c_str(), messageCSV.length());
  Serial.println(messageCSV);
}

void sendMessage()
{
  /*
    message = "Distance travelled: " + String(int(distanceTravelledByTheCar))
              + " Speed: " + String(int(carSpeed))
              + ((nearestObstacleDistance != 100) ? " Object at: " + String(int(nearestObstacleDistance)) : " No Object")
              + " Current mode: " + ((currentMode == MODE_1_Object_Following) ? "Mode1 \n" : "Mode2 \n");

    */

  /*
    message = "Distance travelled: " + String(int((leftPulseCount + rightPulseCount) * 3.142 * radiusOfWheel / EncoderPulsesPerRevolution))
              + " error: " + String(Kp * error)
              + " integral: " + String(Ki * integral)
              + " differential: " + String(Kd * differential)
              + " elapsedTime: " + String(elapsedTime)
              + " Speed: " + String(int(carSpeed))
              + ((nearestObstacleDistance != 100) ? " Object at: " + String(int(nearestObstacleDistance)) : " No Object")
              + " Current mode: " + ((currentMode == MODE_1_Object_Following) ? "Mode1 \n" : "Mode2 \n");
    */

  /*
    message[0] = int(distanceTravelledByTheCar);
    message[1] = int(carSpeed);
    message[2] = int(nearestObstacleDistance);


    ProcessingClient.write(message, sizeof(message));
    */
}

void calculateSpeed()
{
  leftSpeedMmS = double(arcLengthMm / (leftTimeCurrent - leftTimePrev)); // mm/s
  rightSpeedMmS = double(arcLengthMm / (rightTimeCurrent - rightTimePrev));
  averageSpeedMmS = double((leftSpeedMmS + rightSpeedMmS) / 2);

  // Serial.println(averageSpeedMmS);
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
int mapSpeedMmSToPWM(double speed)
{
  int PWMValue = round(map(speed, MinSpeedMmS, MaxSpeedMmS, PWMMin, PWMMax));

  // Serial.println("Speed: ");
  // Serial.println(speed);
  // Serial.println(" | Mapped PWM: ");
  // Serial.println(floor(PWMValue));

  return PWMValue;
}

// Function to move the car forward at a specified speed
void moveForwardatSpeed(double speed)
{
  Serial.print("Max forward at ");
  Serial.println(MaxSpeedMmS);

  Serial.print("Moving forward at ");
  Serial.println(speed);

  // Adjust right motor PWM based on the specified speed
  analogWrite(RightMotorPWM, mapSpeedMmSToPWM(RightWheelCoefficient * speed));

  // Adjust left motor PWM based on the left wheel coefficient
  analogWrite(LeftMotorPWM, mapSpeedMmSToPWM(LeftWheelCoefficient * speed));

  // Switch on
  digitalWrite(RightMotorSwitchActive, HIGH);
  digitalWrite(LeftMotorSwitchActive, HIGH);
}

// Function to stop the car
void stopCar()
{
  // Serial.println("Stopping car");

  // Stop the right motor by setting its PWM to 0
  analogWrite(RightMotorPWM, mapSpeedMmSToPWM(0));

  // Stop the left motor by setting its PWM to 0
  analogWrite(LeftMotorPWM, mapSpeedMmSToPWM(0));

  digitalWrite(RightMotorSwitchActive, LOW);
  digitalWrite(LeftMotorSwitchActive, LOW);
}

// Function to turn the car to the left
void turnLeft()
{
  // Serial.println("Inside turnLeft - Turning left");

  // Adjust the right motor PWM for a left turn
  analogWrite(RightMotorPWM, RightWheelCoefficient * TurnSpeedOuterPulseWidth);

  // Stop the left motor by setting its PWM to 0
  analogWrite(LeftMotorPWM, LeftWheelCoefficient * TurnSpeedInnerPulseWidth);

  digitalWrite(RightMotorSwitchActive, HIGH);
  digitalWrite(LeftMotorSwitchActive, HIGH);
}

// Function to turn the car to the right
void turnRight()
{
  // Serial.println("Inside turnRight - Turning right");

  // Stop the right motor by setting its PWM to 0
  analogWrite(RightMotorPWM, RightWheelCoefficient * TurnSpeedInnerPulseWidth);

  // Adjust the left motor PWM for a right turn
  analogWrite(LeftMotorPWM, LeftWheelCoefficient * TurnSpeedOuterPulseWidth);

  // Switch on
  digitalWrite(RightMotorSwitchActive, HIGH);
  digitalWrite(LeftMotorSwitchActive, HIGH);
}

// Sensor Functions

// Function to measure the closest obstacle distance using the ultrasonic sensor
double closestObstacleUsingSonar()
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

  // Measure the duration of the pulse , only wait for 3000 microseconds
  unsigned long pulseDuration = pulseIn(UltrasonicEchoDetector, HIGH, 3000);

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

  if (currentMode == MODE_1_Object_Following)
  {

    carSpeedAlmostMmS = PIDSpeedObjectFollowing();

    // Serial.println("Inside Object Tracking, changing speed to: " + String(carSpeedMmS));
  }
  else if (currentMode == MODE_2_Speed_Control)
  {

    carSpeedAlmostMmS = PIDSpeedMaintainSpeed();

    // Serial.println("Inside Speed PID change, changing speed to: " + String(carSpeedMmS));
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

    moveForwardatSpeed(carSpeedAlmostMmS);
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

  // connectionSetup();

  // Serial.println("Inside Setup afer connection");

  // Interrupts

  // Serial.println("1 Inside Setup 2");

  attachInterrupt(
      digitalPinToInterrupt(RightEncoder), []()
      {
      rightPulseCount++;
      rightTimePrev = rightTimeCurrent;
      rightTimeCurrent = micros(); },
      RISING);

  attachInterrupt(
      digitalPinToInterrupt(LeftEncoder), []()
      {
      leftPulseCount++;
      leftTimePrev = leftTimeCurrent;
      leftTimeCurrent = micros(); },
      RISING);

  // Serial.println(" 4 Inside Setup 3");

  /*

    while (!ProcessingClient.connected()) {
      // Attempt to accept an incoming client connection on the WiFi server
      ProcessingClient = server.available();

      Serial.print("-");

      delay(100);
    }

    Serial.println("Connected!");
    Serial.println("Connected!");

    */

  Serial.println("Connected!");
}

// Main execution loop function that runs continuously after setup
void loop()
{

  // Delegate the primary control logic
  // Serial.print('.');

  // Serial.println("starting loop");  // Optional debugging statement
  if (loopCounter % 23 == 0)
  {

    checkPositionRelativeToObject();
  }
  else if (loopCounter % 31 == 0)
  {

    // checkServer();
  }
  else if (loopCounter % 700 == 0)
  {

    // Serial.print("Got into the 500th loop");
    distanceTravelledByTheCarCm = (leftPulseCount + rightPulseCount) * 3.142 * radiusOfWheelCm / EncoderPulsesPerRevolution;

    calculateSpeed();

    sendMessageCSV();

    // Reset the loop counter for the next iteration

    loopCounter = 0;
  }

  // Serial.println("Is the obstacleTooClose?");  // Optional debugging statement
  // Serial.println(obstacleTooClose);  // Optional debugging statement

  if (!obstacleTooClose && !StopTheCar)
  {

    // Continue moving and checking infrared sensors if no obstacles are too close
    //  Serial.print("MOVE BRO?");  // Optional debugging statement

    keepMovingCheckingIRSensors();
  }
  else if (!StopTheCar) // obstacleTooClose &&
  {

    delayMicroseconds(3000);

    checkPositionRelativeToObject();
  }

  // Increment the loop counter for each iteration
  loopCounter++;

  // Serial.println("end of loop");
}

