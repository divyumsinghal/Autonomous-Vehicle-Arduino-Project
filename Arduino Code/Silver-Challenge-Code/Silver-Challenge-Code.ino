// Guthub: https://github.com/divyumsinghal/Autonomous-Vehicle-Arduino-Project

// URLs for different car commands sent from processing to arduino

#define startURL 'Z'
#define stopURL 'X'

#define displayHeartURL 'H'
#define displaySmileyURL 'S'
#define displayW5URL 'W'

// Target Speed is send as an integer in ASCII

// Include the library for handling the LED matrix
#include "Arduino_LED_Matrix.h"

// Include the library for WiFi communication using the S3 module
#include <WiFiS3.h>

// Variables

// Pin Definitions
const int LeftIRSensorInput = A1;               // Analog pin for the left infrared sensor
const int RightIRSensorInput = A0;              // Analog pin for the right infrared sensor
const int RightMotorPWM = 10;                   // PWM pin for controlling the right motor speed
const int LeftMotorPWM = 9;                     // PWM pin for controlling the left motor speed
const int RightMotorSwitch1 = 11;               // Motor control pin 1 for the right motor
const int RightMotorSwitch2 = 12;               // Motor control pin 2 for the right motor (changed to 12, used to be 13)
const int LeftMotorSwitch3 = 8;                 // Motor control pin 3 for the left motor
const int LeftMotorSwitch4 = 7;                 // Motor control pin 4 for the left motor
const int UltrasonicTrigger = 5;                // Digital pin for triggering the ultrasonic sensor
const int UltrasonicEchoDetector = 4;           // Digital pin for detecting the echo from the ultrasonic sensor
const int LED_PIN = 13;                         // Digital pin for the LED
int RightMotorSwitchActive = RightMotorSwitch1; // Active Switch for right motor
int LeftMotorSwitchActive = LeftMotorSwitch3;   // Active Switch for Left motor

// Defining encoders
const int RightEncoder = 3;                 // Digital pin for the right motor encoder
const int LeftEncoder = 2;                  // Digital pin for the left motor encoder
volatile unsigned long leftPulseCount = 0;  // Pulse count for the left motor encoder
volatile unsigned long rightPulseCount = 0; // Pulse count for the right motor encoder

// Constants
const double RightWheelCoefficient = 0.87;     // Coefficient for adjusting Right wheel speed
const double LeftWheelCoefficient = 1;         // Coefficient for adjusting left wheel speed
const double MinSpeedCmS = 0;                  // Minimum speed CmS for the car
const double MaxSpeedCmS = 50;                 // Maximum speed CmS for the car
const int PWMMin = 0;                          // Minimum PWM value
const int PWMMax = 140;                        // Maximum PWM value (capping it at 135 instead of 255)
const int turnSpeedOuterPulseWidth = 115;      // Turning speed for outer wheel
const int turnSpeedInnerPulseWidth = 30;       // Turning speed for inner wheel
const int EncoderPulsesPerRevolution = 4;      // Encoder generates 8 pulses per revolution -> 4 rising are tracked
const int CriticalObjectDistance = 10;         // Critical distance for detecting obstacles
const int ObjectFollowingDistance = 20;        // A slightly larger and safer distance
const int Overtime = 51;                       // Return this when sonar takes too long
const double SPEED_OF_SOUND_CM_PER_MS = 0.017; // Conversion factor for microseconds to distance
const double radiusOfWheelCm = 3.5;            // radius of wheel in cm
const double arcLengthCorrection = 0.35;       // correction for speed

// more variables
double nearestObstacleDistance = 100;                                                                                           // Distance to the nearest obstacle from ultrasonic sensor
bool obstacleTooClose = false;                                                                                                  // Flag indicating if an obstacle is too close
double carSpeedAlmostCmS = MaxSpeedCmS;                                                                                         // Current speed of the car
unsigned long loopCounter = 0;                                                                                                  // Counter for obstacle tracking frequency
bool StopTheCarThroughGUI = true;                                                                                               // Control if you want the car to move
double distanceTravelledByTheCarCm = (leftPulseCount + rightPulseCount) * 3.142 * radiusOfWheelCm / EncoderPulsesPerRevolution; // How far have the wheels spun (in cm)
double targetSpeedCmS_MODE_2_Speed_Control_PID = MaxSpeedCmS;                                                                                            // Speed to reach in mode 2

// MODES

// Define an enumeration for modes
enum Modes
{
  MODE_0_Bronze,
  MODE_1_Object_Following, // Object Following -> PID
  MODE_2_Speed_Control_PID     // Speed Control -> Slider on GUI

};

// Variable to store the current mode
Modes currentMode = MODE_2_Speed_Control_PID;

// Speed of Buggy

// Define arc length in millimeters based on wheel radius and angular displacement
double arcLengthCmMilli = (double)(1000 * arcLengthCorrection * 2 * 3.142 * radiusOfWheelCm / 4);

// Variables to store previous and current time for left wheel and right wheel interrupts
volatile unsigned long leftTimePrev = millis();
volatile unsigned long leftTimeCurrent = infinity();

volatile unsigned long rightTimePrev = millis();
volatile unsigned long rightTimeCurrent = infinity();

// Variables to store past time values for left and right wheels
volatile unsigned long leftTimeCurrentPast = leftTimeCurrent;
volatile unsigned long rightTimeCurrentPast = rightTimeCurrent;

// Calculate left and right wheel speeds in millimeters per second (mm/s)
double leftSpeedCmS = (double)(arcLengthCmMilli / (leftTimeCurrent - leftTimePrev)); // mm/s
double rightSpeedCmS = (double)(arcLengthCmMilli / (rightTimeCurrent - rightTimePrev));

// Calculate average speed of both wheels in millimeters per second (mm/s)
double experimentalSpeedCmS = (double)((leftSpeedCmS + rightSpeedCmS) / 2);

void calculateSpeed()
{

  // Update the current time for both left and right measurements
  rightTimeCurrent = (rightTimeCurrent == rightTimeCurrentPast) ? (9 * rightTimeCurrent + millis()) / 10 : rightTimeCurrent;
  leftTimeCurrent = (leftTimeCurrent == leftTimeCurrentPast) ? (9 * leftTimeCurrent + millis()) / 10 : leftTimeCurrent;

  // Keep track of a past time
  leftTimeCurrentPast = leftTimeCurrent;
  rightTimeCurrentPast = rightTimeCurrent;

  // Calculate the speed of the left wheel in millimeters per second (mm/s)
  // using the formula: speed = distance / time
  // where distance is 'arcLengthMm' and time is the time elapsed since the previous measurement
  leftSpeedCmS = (double)(arcLengthCmMilli / (leftTimeCurrent - leftTimePrev)); // mm/s

  // Calculate the speed of the right wheel in millimeters per second (mm/s)
  // using the same formula as for the left wheel
  rightSpeedCmS = (double)(arcLengthCmMilli / (rightTimeCurrent - rightTimePrev));

  // Calculate the average speed of both wheels in millimeters per second (mm/s)
  // by taking the mean of their individual speeds
  experimentalSpeedCmS = (double)((leftSpeedCmS + rightSpeedCmS) / 2);

  // Serial.print("leftSpeedCmS: ");
  // Serial.println(leftSpeedCmS);

  // Serial.println(leftPulseCount);

  // Serial.print("leftTimePrev: ");
  // Serial.println(leftTimePrev);

  // Serial.print("leftTimeCurrent: ");
  // Serial.println(leftTimeCurrent);

  // Serial.print(" rightSpeedCmS: ");
  // Serial.println(rightSpeedCmS);

  // Serial.print("rightPulseCount: ");
  // Serial.println(rightPulseCount);

  // Serial.print("rightTimePrev: ");
  // Serial.println(rightTimePrev);

  // Serial.print("rightTimeCurrent: ");
  // Serial.println(rightTimeCurrent);
}

/*

double experimentalSpeedCmS;
double prevDistanceCm = 0;
double prevtime = 0;

void calculateSpeed() {

  distanceTravelledByTheCarCm = (leftPulseCount + rightPulseCount) * 3.142 * radiusOfWheelCm / EncoderPulsesPerRevolution;

  experimentalSpeedCmS = (double)(1e6) * (distanceTravelledByTheCarCm - prevDistanceCm) / (micros() - prevtime);

  prevDistanceCm = distanceTravelledByTheCarCm;
  prevtime = micros();
}

*/

// PID

// PID for Object Following

// Define PID constants
const double Kp_f_1 = 0.5;            // Proportional gain
const double Ki_f_1 = 0.0000025; // Integral gain
const double Kd_f_1 = 200;            // Derivative gain

// Define variables
double error_f_1 = 0;         // Current error
double previousError_f_1 = 0; // Error in the previous iteration

double integral_f_1 = 0;     // Integral of the error over time
double differential_f_1 = 0; // Derivative of the error

double controlSignal_f_1 = 0; // Control signal output

double currentTime_f_1 = infinity(); // Current time
double previousTime_f_1 = millis();  // Time in the previous iteration
double elapsedTime_f_1 = 0;          // Elapsed time since the previous iteration

// Define function
double PIDObjectFollowing_f_1()
{

  // Time
  currentTime_f_1 = millis();
  elapsedTime_f_1 = (double)(currentTime_f_1 - previousTime_f_1);

  // Calculate error is proportional
  error_f_1 = (double)(nearestObstacleDistance - ObjectFollowingDistance);

  // Update integral
  integral_f_1 += (error_f_1 < 10) ? (double)(error_f_1 * elapsedTime_f_1) : 0;

  // Update differential
  differential_f_1 = (double)((error_f_1 - previousError_f_1) / elapsedTime_f_1);

  // Calculate control signal
  controlSignal_f_1 = (double)(Kp_f_1 * error_f_1 + Ki_f_1 * integral_f_1 + Kd_f_1 * differential_f_1);

  // Update previous error & time
  previousError_f_1 = error_f_1;
  previousTime_f_1 = currentTime_f_1;

  // Calculate next speed
  double nextSpeed_f_1 = carSpeedAlmostCmS + controlSignal_f_1;

  // Ensure the next speed is within bounds
  nextSpeed_f_1 = constrain(nextSpeed_f_1, MinSpeedCmS, MaxSpeedCmS);

  // Serial.println("elapsedTime_f_1 = " + String(elapsedTime_f_1));
  // Serial.println("targetSpeedCmS = " + String(targetSpeedCmS));
  // Serial.println("experimentalSpeedCmS = " + String(experimentalSpeedCmS));
  // Serial.println("carSpeedAlmostCmS = " + String(carSpeedAlmostCmS));
  // Serial.println("error_f_1: " + String(error_f_1));
  // Serial.println("differential_f_1: " + String(differential_f_1 * Kd_f_1));
  // Serial.println("controlSignal_f_1: " + String(controlSignal_f_1));
  // Serial.println("nextSpeed_f_1: " + String(nextSpeed_f_1) + "\n");

  return nextSpeed_f_1;
}

// PID for Speed Control

// Define PID constants

const double Kp_sc_2 = 0.025;      // Proportional gain
const long double Ki_sc_2 = 5e-10; // Integral gain
const double Kd_sc_2 = 2e-8;       // Derivative gain

// Define variables
double error_sc_2 = 0;         // Current error
double previousError_sc_2 = 0; // Error in the previous iteration

double integral_sc_2 = 0;     // Integral of the error over time
double differential_sc_2 = 0; // Derivative of the error

double controlSignal_sc_2 = 0; // Control signal output

unsigned long currentTime_sc_2 = infinity(); // Current time
unsigned long previousTime_sc_2 = millis();  // Time in the previous iteration
double elapsedTime_sc_2 = 0;                 // Elapsed time since the previous iteration

// Function to calculate PID for maintaining speed
double PIDMaintainSpeed_sc_2()
{

  calculateSpeed();

  // Time
  currentTime_sc_2 = millis();
  elapsedTime_sc_2 = (double)(currentTime_sc_2 - previousTime_sc_2);

  // Calculate error
  error_sc_2 = (double)(targetSpeedCmS_MODE_2_Speed_Control_PID - experimentalSpeedCmS);

  // Serial.println("Error_sc_2 = " + String(error_sc_2));

  // Update integral
  integral_sc_2 += (double)(error_sc_2 * elapsedTime_sc_2);

  // Update differential
  differential_sc_2 = (double)((error_sc_2 - previousError_sc_2) / elapsedTime_sc_2);

  // Calculate control signal
  controlSignal_sc_2 = (double)(Kp_sc_2 * error_sc_2 + Ki_sc_2 * integral_sc_2 + Kd_sc_2 * differential_sc_2);

  // Update previous error & time
  previousError_sc_2 = error_sc_2;
  previousTime_sc_2 = currentTime_sc_2;

  // Calculate next speed
  double nextSpeed_sc_2 = carSpeedAlmostCmS + controlSignal_sc_2;

  // Ensure the next speed is within bounds
  nextSpeed_sc_2 = constrain(nextSpeed_sc_2, MinSpeedCmS, MaxSpeedCmS);

  // Serial.println("elapsedTime_sc_2 = " + String(elapsedTime_sc_2));
  // Serial.println("targetSpeedCmS = " + String(targetSpeedCmS));
  // Serial.println("experimentalSpeedCmS = " + String(experimentalSpeedCmS));
  // Serial.println("carSpeedAlmostCmS = " + String(carSpeedAlmostCmS));
  // Serial.println("error_sc_2: " + String(error_sc_2));
  // Serial.println("differential_sc_2: " + String(differential_sc_2 * Kd_sc_2));
  // Serial.println("controlSignal_sc_2: " + String(controlSignal_sc_2));
  // Serial.println("nextSpeed_sc_2: " + String(nextSpeed_sc_2) + "\n");

  return nextSpeed_sc_2;
}

// Wifi

// Wifi Details
char ssid[] = "w5shouldget100";
char pass[] = "bestgroupfr123";

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

  data = ProcessingClient.read();

  switch (data)
  {
  case startURL:

    // Serial.println(data);

    StopTheCarThroughGUI = false;

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
    // Handle speed Command

    // Serial.print(data);

    // Check if the current mode is MODE_2_Speed_Control
    if (currentMode == MODE_2_Speed_Control_PID)
    {

      // Calculate the desired speed based on the received data
      // data - '0' converts from ASCII to integer, then add 1 and multiply by 3
      int setSpeed = (data - '0' + 1) * 5;

      // Check if the calculated speed is within acceptable range
      if (setSpeed > MinSpeedCmS && setSpeed <= MaxSpeedCmS)
      {

        // Update the target speed and the almost car speed with the new value

        targetSpeedCmS_MODE_2_Speed_Control_PID = setSpeed;
        carSpeedAlmostCmS = targetSpeedCmS_MODE_2_Speed_Control_PID;

        // Print a message debugging the change in speed (commented out)
        // Serial.println("Inside Mode 2, Changing speed to: " + String(targetSpeedCmS));
      }
    }

    break;
  }
}

void sendMessageCSV()
{

  // Construct a comma-separated value (CSV) message containing various data
  // The data includes: distance travelled by the car in centimeters, average speed in millimeters per second,
  // and distance to the nearest obstacle in some unit (possibly centimeters).
  messageCSV = String(int(distanceTravelledByTheCarCm)) + "," + String(int(experimentalSpeedCmS)) + "," + String(int(nearestObstacleDistance)) + "," + String(int(2 * carSpeedAlmostCmS)) + "," + String(int(currentMode)) + " \n ";

  // Write the constructed CSV message to the Processing client
  // The ProcessingClient object is assumed to have a write function that accepts a character array and its length
  ProcessingClient.write(messageCSV.c_str(), messageCSV.length());

  // Print the constructed CSV message (commented out)
  // Serial.println(messageCSV);
}

void sendMessage()
{
  /*
    message = "Distance travelled: " + String(int(distanceTravelledByTheCar))
              + " Speed: " + String(int(    carSpeedAlmostCmS)
              + ((nearestObstacleDistance != 100) ? " Object at: " + String(int(nearestObstacleDistance)) : " No Object")
              + " Current mode: " + ((currentMode == MODE_1_Object_Following) ? "Mode1 \n" : "Mode2 \n");

    */

  /*
    message = "Distance travelled: " + String(int((leftPulseCount + rightPulseCount) * 3.142 * radiusOfWheel / EncoderPulsesPerRevolution))
              + " error: " + String(Kp * error)
              + " integral: " + String(Ki * integral)
              + " differential: " + String(Kd * differential)
              + " elapsedTime: " + String(elapsedTime)
              + " Speed: " + String(int(    carSpeedAlmostCmS)
              + ((nearestObstacleDistance != 100) ? " Object at: " + String(int(nearestObstacleDistance)) : " No Object")
              + " Current mode: " + ((currentMode == MODE_1_Object_Following) ? "Mode1 \n" : "Mode2 \n");
    */

  /*
    message[0] = int(distanceTravelledByTheCar);
    message[1] = int(    carSpeedAlmostCmS;
    message[2] = int(nearestObstacleDistance);



    ProcessingClient.write(message, sizeof(message));
    */
}

// Matrix Handling

ArduinoLEDMatrix matrix;

// Predefined patterns for the LED matrix

// Smiley pattern
const uint32_t happy[3] = {

    0x19819,
    0x80000001,
    0x81f8000

};

// Heart pattern
const uint32_t heart[3] = {

    0x3184a444,
    0x44042081,
    0x100a0040

};

// W5 pattern
const uint32_t W5[4] = {

    0x82f82882,
    0x892fba1e,
    0xe1ee1c6f,
    66

};

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
  // Return the distance or Overtime if no pulse received within 3000 microseconds

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

    // If obstacle is not too close, start the car
    // or keep it moving if it is already moving

    obstacleTooClose = false;

    // The PID runs after this
  }

  if (nearestObstacleDistance < Overtime)
  {

    currentMode = MODE_1_Object_Following;
  }
  else
  {

    currentMode = MODE_2_Speed_Control_PID;
  }

  // Serial.println("Inside Object Tracking, changing speed to: " + String(carSpeedAlmostCmS));
}

// Keeping the car moving while checking infrared sensors for tracking
void keepMovingCheckingIRSensors()
{

  // Serial.println("got inside keepMovingCheckingIRSensors");

  int irLeftValue = digitalRead(LeftIRSensorInput);
  int irRightValue = digitalRead(RightIRSensorInput);

  // Move straight for debugging
  // int irLeftValue = LOW;
  // int irRightValue = LOW;

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

    moveForwardatSpeed(carSpeedAlmostCmS);
  }

  // Serial.println("end of keepMovingCheckingIRSensors");
}

// Functions needed for Movement

inline int mapSpeedCmSToPWM(double speed);

// Mapping speed to PWM values
inline int mapSpeedCmSToPWM(double speed)
{

  int PWMValue = round(
      map(speed,
          MinSpeedCmS,
          MaxSpeedCmS,
          PWMMin,
          PWMMax));

  // Serial.println("Speed: ");
  // Serial.println(speed);
  // Serial.println(" | Mapped PWM: ");
  // Serial.println(floor(PWMValue));

  return PWMValue;
}

// Function to move the car forward at a specified speed
void moveForwardatSpeed(double speed)
{

  // Serial.print("Max forward at ");
  // Serial.println(MaxSpeedCmS);

  // Serial.print("Moving forward at ");
  // Serial.println(speed);

  // Adjust right motor PWM based on the specified speed
  analogWrite(
      RightMotorPWM,
      mapSpeedCmSToPWM(RightWheelCoefficient * speed));

  // Adjust left motor PWM based on the left wheel coefficient
  analogWrite(
      LeftMotorPWM,
      mapSpeedCmSToPWM(LeftWheelCoefficient * speed));

  // Switch on
  digitalWrite(RightMotorSwitchActive, HIGH);

  digitalWrite(LeftMotorSwitchActive, HIGH);
}

// Function to stop the car
void stopCar()
{

  // Serial.println("Stopping car");

  // Stop the right motor by setting its PWM to 0
  analogWrite(
      RightMotorPWM,
      mapSpeedCmSToPWM(0));

  // Stop the left motor by setting its PWM to 0
  analogWrite(
      LeftMotorPWM,
      mapSpeedCmSToPWM(0));

  // Switch off

  digitalWrite(RightMotorSwitchActive, LOW);

  digitalWrite(LeftMotorSwitchActive, LOW);
}

// Function to turn the car to the left
void turnLeft()
{

  // Serial.println("Inside turnLeft - Turning left");

  // Adjust the right motor PWM for a left turn

  // Using direct PWM instead of mapped speeds as the car doesnt
  // turn at too low speeds or skids at too high

  analogWrite(
      RightMotorPWM,
      RightWheelCoefficient * turnSpeedOuterPulseWidth);

  // Stop the left motor
  analogWrite(
      LeftMotorPWM,
      LeftWheelCoefficient * turnSpeedInnerPulseWidth);

  // Switch on

  digitalWrite(RightMotorSwitchActive, HIGH);

  digitalWrite(LeftMotorSwitchActive, HIGH);
}

// Function to turn the car to the right
void turnRight()
{

  // Serial.println("Inside turnRight - Turning right");

  // Stop the right motor by setting its PWM

  // Using direct PWM instead of mapped speeds as the car doesnt
  // turn at too low speeds or skids at too high

  analogWrite(
      RightMotorPWM,
      RightWheelCoefficient * turnSpeedInnerPulseWidth);

  // Adjust the left motor PWM for a right turn
  analogWrite(
      LeftMotorPWM,
      LeftWheelCoefficient * turnSpeedOuterPulseWidth);

  // Switch on

  digitalWrite(RightMotorSwitchActive, HIGH);

  digitalWrite(LeftMotorSwitchActive, HIGH);
}

// Arduino functions

// Initialization function executed once at the start of the program
void setup()
{

  Serial.begin(9600); // Initialize Serial communication with a baud rate of 9600

  pinMode(LeftIRSensorInput, INPUT);  // Configure the left infrared sensor pin as an input
  pinMode(RightIRSensorInput, INPUT); // Configure the right infrared sensor pin as an input

  pinMode(UltrasonicTrigger, OUTPUT);     // Configure the ultrasonic trigger pin as an output
  pinMode(UltrasonicEchoDetector, INPUT); // Configure the ultrasonic echo detector pin as an input

  pinMode(LeftMotorSwitch3, OUTPUT);  // Configure the left motor control pin 3 as an output
  pinMode(LeftMotorSwitch4, OUTPUT);  // Configure the left motor control pin 4 as an output
  pinMode(RightMotorSwitch1, OUTPUT); // Configure the right motor control pin 1 as an output
  pinMode(RightMotorSwitch2, OUTPUT); // Configure the right motor control pin 2 as an output

  digitalWrite(RightMotorSwitch1, LOW); // Initially set both motors to be swtiched off
  digitalWrite(RightMotorSwitch2, LOW); // Initially set both motors to be swtiched off
  digitalWrite(LeftMotorSwitch3, LOW);  // Initially set both motors to be swtiched off
  digitalWrite(LeftMotorSwitch4, LOW);  // Initially set both motors to be swtiched off

  pinMode(LeftMotorPWM, OUTPUT);  // Configure the left motor PWM pin as an output
  pinMode(RightMotorPWM, OUTPUT); // Configure the right motor PWM pin as an output

  pinMode(LeftEncoder, INPUT_PULLUP);  // Configure the left motor encoder pin with pull-up resistor enabled
  pinMode(RightEncoder, INPUT_PULLUP); // Configure the right motor encoder pin with pull-up resistor enabled
                                       // & logic inversion (a 20 k resistor in parallel for impedence control)

  // Initialize the LED matrix for further use
  matrix.begin();

  // Setup Connection

  Serial.println("Inside Setup before connection");

  Serial.println("Setting up!");

  connectionSetup();

  Serial.println("Inside Setup afer connection");

  // Interrupts

  Serial.println("1 Inside Setup 2");

  // Coding in the Interrupts

  // This sets up an ISR for the RightEncoder pin.
  // When a rising edge is detected on this pin (which typically corresponds to a notch passing by the encoder sensor),
  // the provided lambda function is executed

  attachInterrupt(

      digitalPinToInterrupt(RightEncoder), []()
      {
      // Increment the pulse count for the right encoder
      rightPulseCount++;

      // Update the previous time to the current time
      rightTimePrev = rightTimeCurrent;

      // Update the current time
      rightTimeCurrent = millis(); },

      RISING);

  // Similar to the previous block, this sets up an ISR for the LeftEncoder pin.
  // When a rising edge is detected (indicating a notch passing by the sensor),
  // the provided lambda function is executed.

  attachInterrupt(

      digitalPinToInterrupt(LeftEncoder), []()
      {
      // Increment the pulse count for the left encoder
      leftPulseCount++;

      // Update the previous time to the current time
      leftTimePrev = leftTimeCurrent;

      // Update the current time
      leftTimeCurrent = millis(); },

      RISING);

  // These ISRs are commonly used in motor control applications to accurately
  // measure the speed and distance traveled by a rotating wheel.

  Serial.println(" 4 Inside Setup 3");

  // This line initiates a while loop that continues until a connection is established with the client.
  // It checks if the client is not connected.

  while (!ProcessingClient.connected())
  {

    // Attempt to accept an incoming client connection on the WiFi server
    ProcessingClient = server.available();

    // used for debugging purposes to indicate that the program is still attempting to establish a connection
    Serial.print("-");

    // This line introduces a delay of 100 milliseconds.
    // It's a common practice to add a delay when waiting for a connection attempt to prevent
    // excessive CPU usage and to allow other tasks to be performed during the waiting period.

    delay(100);
  }

  Serial.println("Connected!");
  Serial.println("Connected!");
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

    // Serial.println("Got into the 23rd loop");

    checkPositionRelativeToObject();

    if (currentMode == MODE_1_Object_Following)
    {

      carSpeedAlmostCmS = PIDObjectFollowing_f_1();

      // Serial.println("Inside Object Tracking, changing speed to: " + String(carSpeedAlmostCmS));
    }
    else if (currentMode == MODE_2_Speed_Control_PID)
    {

      carSpeedAlmostCmS = PIDMaintainSpeed_sc_2();

      // Serial.println("Inside Speed PID change, changing speed to: " + String(carSpeedAlmostCmS));
    }
  }
  else if (loopCounter % 31 == 0)
  {

    checkServer();
  }
  else if (loopCounter % 700 == 0)
  {

    // Serial.println("Got into the 700th loop");

    // This line calculates the distance travelled by the car in centimeters.
    // It multiplies the sum of the pulse counts from the left and right encoders by a constant
    // representing the circumference of the wheel (3.142 * radiusOfWheelCm) divided by the
    // number of pulses per revolution of the encoder (EncoderPulsesPerRevolution), and assigns the result
    // to the variable distanceTravelledByTheCarCm.

    distanceTravelledByTheCarCm = (leftPulseCount + rightPulseCount) * 3.142 * radiusOfWheelCm / EncoderPulsesPerRevolution;

    // calculates the speed of the car based on set parameters & measurements
    calculateSpeed();

    // This function sends a message, containing data such as distance travelled and car speed,
    // in Comma-Separated Values (CSV) format, which is commonly used for data exchange between systems
    sendMessageCSV();

    // Reset the loop counter for the next iteration
    loopCounter = 0;

    // reset the integrals
    integral_sc_2 /= 50;
    integral_f_1 /= 50;
  }

  // Serial.println("Is the obstacleTooClose?");  // Optional debugging statement
  // Serial.println(obstacleTooClose);            // Optional debugging statement

  if (!obstacleTooClose && !StopTheCarThroughGUI)
  {

    // Continue moving and checking infrared sensors if no obstacles are too close
    // Serial.print("MOVE BRO?");  // Optional debugging statement

    keepMovingCheckingIRSensors();
  }
  else if (!StopTheCarThroughGUI) // obstacleTooClose &&
  {

    delayMicroseconds(3000);

    checkPositionRelativeToObject();
  }

  // Increment the loop counter for each iteration
  loopCounter++;

  // Serial.println("end of loop");
}
