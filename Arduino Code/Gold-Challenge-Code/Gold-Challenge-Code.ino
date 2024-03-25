/*
 * @file Gold-Challenge-Code.ino
 * @brief Arduino code for an Autonomous Vehicle project.
 *
 * This code controls an autonomous vehicle using an Arduino board. It includes various functionalities such as WiFi communication, LED matrix handling, obstacle detection using ultrasonic sensor, object following using PID control, and speed control. The code also includes definitions for different car commands and pin assignments for various components.
 *
 * GitHub Repository: [Autonomous-Vehicle-Arduino-Project](https://github.com/divyumsinghal/Autonomous-Vehicle-Arduino-Project)
 *
 * Author: Divyum Singhal
 */

// 2753 milliseconds/10000 loop

// URLs for different car commands sent from processing to arduino

#define startURL 'Z'
#define stopURL 'X'

#define displayHeartURL 'H'
#define displaySmileyURL 'S'
#define displayW5URL 'W'

#define Mode0URL 'A'
#define Mode2URL 'C'

// Target Speed is send as an integer in ASCII

// Include the library for handling the LED matrix
#include "Arduino_LED_Matrix.h"
#include <ArduinoGraphics.h>

// Include the library for WiFi communication using the S3 module
#include <WiFiS3.h>

// Include the library for HuskyLens reading
#include "HUSKYLENS.h"

// Include the library for setting up Wire
#include "SoftwareSerial.h"

// #include "Talkie.h"
// #include "Vocab_US_Large.h"

// Talkie voice;

// Variables

// Pin Definitions
const int LeftIRSensorInput = A1;     // Analog pin for the left infrared sensor
const int RightIRSensorInput = A0;    // Analog pin for the right infrared sensor
const int RightMotorPWM = 10;         // PWM pin for controlling the right motor speed
const int LeftMotorPWM = 9;           // PWM pin for controlling the left motor speed
const int RightMotorSwitch1 = 11;     // Motor control pin 1 for the right motor
const int RightMotorSwitch2 = 12;     // Motor control pin 2 for the right motor (changed to 12, used to be 13)
const int LeftMotorSwitch3 = 8;       // Motor control pin 3 for the left motor
const int LeftMotorSwitch4 = 7;       // Motor control pin 4 for the left motor
const int UltrasonicTrigger = 5;      // Digital pin for triggering the ultrasonic sensor
const int UltrasonicEchoDetector = 4; // Digital pin for detecting the echo from the ultrasonic sensor
const int LED_PIN = 13;               // Digital pin for the LED
const int SPEAKER_PIN = 6;            // Pin for Speaker

/**
 * @brief The active switch for the right motor.
 *
 * This variable holds the value of the active switch for the right motor. It is used to determine which switch is currently active for controlling the right motor. The value of this variable should be set to the appropriate switch pin number.
 * If we need to move forward or backward, we need to set the active switch to the appropriate value.
 *
 * @note The value of this variable should be set to one of the predefined switch pin numbers, such as RightMotorSwitch1, RightMotorSwitch2, etc.
 */

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
const int PWMMax = 160;                        // Maximum PWM value (capping it at 135 instead of 255)
const int EncoderPulsesPerRevolution = 4;      // Encoder generates 8 pulses per revolution -> 4 rising are tracked
const int CriticalObjectDistance = 10;         // Critical distance for detecting obstacles
const int ObjectFollowingDistance = 20;        // A slightly larger and safer distance
const int Overtime = 51;                       // Return this when sonar takes too long
const double SPEED_OF_SOUND_CM_PER_MS = 0.017; // Conversion factor for microseconds to distance
const double radiusOfWheelCm = 3.5;            // radius of wheel in cm
const double arcLengthCorrection = 0.35;       // correction for speed
const int WaitSound = 1000;                    // Bad Sound in Hertz
const int GoodSound = 20;                      // Good Sound in Hertz
const int DangerSound = 100000;                // More Sound in Hertz
const int Sound = 10000;

// more variables
double nearestObstacleDistance = 100;      // Distance to the nearest obstacle from ultrasonic sensor
bool obstacleTooClose = false;             // Flag indicating if an obstacle is too close
double carSpeedAlmostCmS = MaxSpeedCmS;    // Current speed of the car
unsigned long loopCounter = 0;             // Counter for obstacle tracking frequency
bool StopTheCarThroughGUI = true;          // Control if you want the car to move
bool StopTheCarThroughLens = false;        // Control if you want the car to move
double distanceTravelledByTheCarCm = 0;    // How far have the wheels spun (in cm)
double targetSpeedCmS = MaxSpeedCmS;       // Speed to reach in mode 2
bool leftIRSensorSwitchedOnByLens = false; // Switch on or off IR sensors using husky lens
bool rightIRSensorSwitchedOnByLens = true; // Switch on or off IR sensors using husky lens
int TurnSpeedOuterPulseWidth = 115;        // Turning speed for outer wheel
int TurnSpeedInnerPulseWidth = 30;         // Turning speed for inner wheel

// Define Matrix

ArduinoLEDMatrix matrix;

// MODES

// Define an enumeration for modes
enum Modes
{

  MODE_0_Speed_Set_by_Lens, // Keeps moving at set speed
  MODE_1_Object_Following,  // Object Following -> PID
  MODE_2_Speed_Control      // Speed Control -> Slider on GUI

};

// Variable to store the current mode
Modes currentMode = MODE_2_Speed_Control;
Modes setMode = MODE_2_Speed_Control;

// variables for Speed of Car

// Define arc length in millimeters based on wheel radius and angular displacement
const double arcLengthCmMilli = (double)(1000 * arcLengthCorrection * 2 * 3.142 * radiusOfWheelCm / EncoderPulsesPerRevolution);

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

// Calculate the speed of the Car

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

// PID

// PID for Object Following

// Define PID constants for object following

const double Kp_f_1 = 0.5;            // Proportional gain
long const double Ki_f_1 = 0.0000025; // Integral gain
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
/**
 * Calculates the output of the PID controller for object following.
 *
 * This function implements a PID (Proportional-Integral-Derivative) controller
 * to control the movement of an autonomous vehicle while following an object.
 * It takes into account the current position and velocity of the vehicle, as well
 * as the desired position and velocity of the object being followed.
 *
 * The PID controller uses three components to calculate the output:
 * - Proportional (P) component: This component is proportional to the error between
 *   the current position and the desired position. It helps steer the vehicle towards
 *   the object being followed.
 * - Integral (I) component: This component takes into account the cumulative error
 *   over time. It helps eliminate steady-state errors and biases.
 * - Derivative (D) component: This component considers the rate of change of the error.
 *   It helps dampen the response of the controller and improve stability.
 *
 * The function returns the calculated output of the PID controller, which can be used
 * to control the movement of the autonomous vehicle.
 *
 * @return The output of the PID controller for object following.
 */
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

// Define PID constants for speed Control

const double Kp_sc_2 = 0.025;      // Proportional gain
const long double Ki_sc_2 = 5e-10; // integral gain
const double Kd_sc_2 = 2e-8;       // Derivative gain

// Define variables
double error_sc_2 = 0;         // Current error
double previousError_sc_2 = 0; // Error in the previous iteration

double integral_sc_2 = 0;     // integral of the error over time
double differential_sc_2 = 0; // Derivative of the error

double controlSignal_sc_2 = 0; // Control signal output

unsigned long currentTime_sc_2 = infinity(); // Current time
unsigned long previousTime_sc_2 = millis();  // Time in the previous iteration
double elapsedTime_sc_2 = 0;                 // Elapsed time since the previous iteration

// Function to calculate PID for maintaining speed
/**
 * @brief This function implements a PID controller to maintain a desired speed.
 *
 * The PIDMaintainSpeed_sc_2 function calculates the control signal required to maintain a desired speed
 * using a Proportional-Integral-Derivative (PID) controller. The function takes into account the current
 * speed and the desired speed, and calculates the control signal based on the error between the two.
 *
 *
 * @return The control signal calculated by the PID controller.
 */
double PIDMaintainSpeed_sc_2()
{

  calculateSpeed();

  // Time
  currentTime_sc_2 = millis();
  elapsedTime_sc_2 = (double)(currentTime_sc_2 - previousTime_sc_2);

  // Calculate error
  error_sc_2 = (double)(targetSpeedCmS - experimentalSpeedCmS);

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

// Huskly lens

HUSKYLENS huskylens;

// Define an enumeration for Tags
enum TAG
{

  TAG_0,            // Nothing
  TAG_1_Start,      // Start
  TAG_2_Stop,       // Stop
  TAG_3_heart,      // heart
  TAG_4_Smiley,     // Smiley
  TAG_5_W5,         // W5
  TAG_6_Turn_Left,  // Turn Left
  TAG_7_Turn_Right, // Turn Right
  TAG_8_Slow_Down,  // Slow Down
  buffer,
  TAG_9_Speed_Up // Speed UP

};

TAG huskySaw = TAG_0;
TAG lastTagSeen = TAG_0;

// Setup Husky Lens

/**
 * @brief Sets up the HuskyLens module for communication with the Arduino board.
 *
 * This function initializes the necessary settings and configurations to establish
 * communication between the Arduino board and the HuskyLens module. It should be called
 * before any other functions that interact with the HuskyLens module.
 *
 * @details The HuskyLens module is a vision sensor that can detect and track objects
 * using various algorithms. This function prepares the Arduino board to receive data
 * from the HuskyLens module and control its operations.
 *
 * @note Make sure to connect the HuskyLens module to the appropriate pins on the Arduino
 * board before calling this function.
 *
 * @see huskyLensReadData()
 * @see huskyLensSendCommand()
 *
 * @return void
 */
void huskyLensSetup()
{

  // Initialize the I2C communication bus
  Wire.begin();

  // Attempt to initialize communication with the Huskylens using I2C
  while (!huskylens.begin(Wire))
  {

    // Print a failure message to the Serial Monitor
    Serial.println(F("Begin failed!"));

    // Provide troubleshooting suggestions
    Serial.println(F("1. Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
    Serial.println(F("2. Please recheck the connection."));

    tone(SPEAKER_PIN, WaitSound);

    // Introduce a brief delay before retrying initialization
    delay(1000);
  }

  if (!huskylens.isLearned())
  {
    Serial.println("Nothing learned, teach me first !");
  }
}

// Take a huskylens result and print out the x/y coordinates and other useful properties.
void printResult(HUSKYLENSResult result)
{
  if (result.command == COMMAND_RETURN_BLOCK)
  {
    Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
  }
  else if (result.command == COMMAND_RETURN_ARROW)
  {
    Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
  }
  else
  {
    Serial.println("Object unknown!");
  }
}

/**
 * @brief Asks the Husky for input and performs a specific action based on the response.
 *
 * This function is responsible for interacting with the Husky and performing a specific action
 * based on the response received. It prompts the Husky for input and then evaluates the response
 * to determine the appropriate action to take.
 *
 * The logic of this function involves sending a request to the Husky for input and waiting for
 * the response. Once the response is received, it is processed to determine the action to be taken.
 * The specific action may vary depending on the implementation of this function.
 *
 * @note This function assumes that the Husky is connected and ready to receive input.
 *
 * @return void
 */
void askHusky()
{

  // First, check that we have the huskylens connected...
  if (huskylens.request() && huskylens.isLearned() && huskylens.available())
  {

    // OK, we have some blocks to process. available() will return the number of blocks to work through.
    // fetch them using read(), one at a time, until there are none left. Each block gets given to
    // my printResult() function to be printed out to the serial port.
    HUSKYLENSResult result = huskylens.read();

    // printResult(result);

    huskySaw = static_cast<TAG>(result.ID);

    switch (huskySaw)
    {

      // Serial.print("Husky lens saw a tag: ");
      // Serial.println(huskySaw);

    case TAG_1_Start:

      // Serial.println(huskySaw);
      StopTheCarThroughLens = false;
      // Serial.println(huskySaw);

      break;

    case TAG_2_Stop:

      // Serial.println(huskySaw);
      StopTheCarThroughLens = true;
      stopCar();

      break;

    case TAG_3_heart:

      // Serial.println(huskySaw);
      displayHeart();

      break;

    case TAG_4_Smiley:

      // Serial.println(huskySaw);
      displaySmiley();

      break;

    case TAG_5_W5:

      // Serial.println(huskySaw);
      displayW5();

      break;

    case TAG_6_Turn_Left:

      leftIRSensorSwitchedOnByLens = true;
      rightIRSensorSwitchedOnByLens = false;

      break;

    case TAG_7_Turn_Right:

      leftIRSensorSwitchedOnByLens = false;
      rightIRSensorSwitchedOnByLens = true;

      break;

    case TAG_8_Slow_Down:

      lastTagSeen = TAG_8_Slow_Down;

      // Serial.println(huskySaw);

      break;

    case TAG_9_Speed_Up:

      lastTagSeen = TAG_9_Speed_Up;

      // Serial.println(huskySaw);

      break;

    default:

      // Handle unknown command
      // Serial.print(huskySaw);
      break;
    }
  }
  else
  {

    // Serial.print("Husky lens did not see a tag: ");
    // Serial.println(lastTagSeen);

    switch (lastTagSeen)
    {
    case TAG_8_Slow_Down:

      lastTagSeen = TAG_0;

      carSpeedAlmostCmS = MaxSpeedCmS / 2;

      // Serial.println(huskySaw);

      break;

    case TAG_9_Speed_Up:

      lastTagSeen = TAG_0;
      carSpeedAlmostCmS = MaxSpeedCmS;

      // Serial.println(huskySaw);

      break;
    }
  }
}

// Define an enumeration for Faces
enum Face
{

  No_face, // no need
  Divyum   // me

};

/**
 * Function to perform login using HuskyLens face recognition.
 * This function continuously checks for recognized faces until a valid login is achieved.
 * @return void
 */
void huskyLensLogin()
{
  bool login = false; // Flag to indicate whether login is successful

  // Continue attempting login until successful
  while (!login)
  {

    tone(SPEAKER_PIN, WaitSound);

    // Check if HuskyLens is ready and a face is learned and available
    if (huskylens.request() && huskylens.isLearned() && huskylens.available())
    {
      // Read the result from HuskyLens
      HUSKYLENSResult result = huskylens.read();

      // Convert the ID to a Face object
      Face huskyFace = static_cast<Face>(result.ID); // Casting the ID from result

      // Check if the recognized face matches the expected user
      if (huskyFace == Divyum)
      {
        // Set login flag to true
        login = true;

        // Display welcome message and instructions
        Serial.println("Welcome Divyum Singhal!");
        Serial.println("Enjoy your Drive!");
        Serial.println("Please follow the driving rules!");
        Serial.println("Please switch the HuskyLens to tag recognition mode!");

        tone(SPEAKER_PIN, GoodSound);
      }
      else
      {
        // Notify user that the recognized face is not authorized
        Serial.println("Face Not Recognised, Please go away!");

        tone(SPEAKER_PIN, DangerSound);

        matrix.loadFrame(LEDMATRIX_EMOJI_SAD);

        // Delay to avoid continuous processing
        delay(500);
      }
    }
  }
}

// Wifi

// Wifi Details
// char ssid[] = "w5shouldget100";
// char pass[] = "bestgroupfr123";

// The following line declares a character array variable named 'ssid' and 'pass' to store the Wi-Fi network name (SSID) and password.
char ssid[] = "giveW5fullmarks";
char pass[] = "password123";

// Declare an instance of the WiFiServer class named 'server'
WiFiServer server(5200);

// Declare a client (needs to be available globally for 2 way communication across)
WiFiClient WebClient;

// Messages
String messageCSV;

// Data
char data;

// WiFi Setup

// Connection Setup
/**
 * Sets up the connection for the autonomous vehicle.
 * This function initializes the necessary components and establishes the connection with the vehicle.
 * It should be called once at the beginning of the program.
 */
void connectionSetup()
{

  Serial.println("Inside connection setup");

  // Initiate a connection to the WiFi network using the provided SSID and password
  WiFi.beginAP(ssid, pass);

  delay(1000);

  // Obtain the local IP address assigned to the Arduino on the WiFi network
  IPAddress ip = WiFi.localIP();

  // Print a label indicating the following content on the Serial Monitor
  Serial.print("IP Address:");

  // Print the obtained IP address to the Serial Monitor
  Serial.println(ip);

  // Start the WiFiServer to listen for incoming connections on the specified port
  server.begin();
}

/**
 * @brief Connects the client to a server.
 *
 * This function establishes a connection between the client and a server.
 * It is responsible for handling the necessary steps to establish the connection,
 * such as initializing network settings and establishing a socket connection.
 *
 * @note This function assumes that the necessary network settings have been properly configured.
 *
 * @return void
 */
void connectClient()
{

  while (!WebClient.connected())
  {

    // Attempt to accept an incoming client connection on the WiFi server
    WebClient = server.available();

    // used for debugging purposes to indicate that the program is still attempting to establish a connection
    Serial.print("-");

    tone(SPEAKER_PIN, WaitSound);

    // This line introduces a delay of 100 milliseconds.
    // It's a common practice to add a delay when waiting for a connection attempt to prevent
    // excessive CPU usage and to allow other tasks to be performed during the waiting period.

    delay(200);
  }

  tone(SPEAKER_PIN, GoodSound);
}

// Check client connection
/**
 * @brief Checks the server for incoming commands and performs corresponding actions.
 *
 * This function reads data from the WebClient and performs different actions based on the received data.
 * If the received data matches any of the predefined URLs, the function executes the corresponding action.
 * If the received data is a speed command and the current mode is MODE_2_Speed_Control, the function calculates
 * the desired speed based on the received data and updates the target speed and the almost car speed.
 *
 * @note This function assumes that the WebClient has been initialized and is connected to a server.
 */
void checkServer()
{

  // Serial.print('_');

  // Send a "Hello Client" message to the connected client
  //("Hello Client");

  data = WebClient.read();

  //  Serial.print(data);

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

  case Mode0URL:

    setMode = MODE_0_Speed_Set_by_Lens;

    break;

  case Mode2URL:

    setMode = MODE_2_Speed_Control;

    break;

  default:
    // Handle speed Command

    // Serial.print(data);

    // Check if the current mode is MODE_2_Speed_Control
    if (currentMode == MODE_2_Speed_Control)
    {

      // Calculate the desired speed based on the received data
      // data - '0' converts from ASCII to integer, then add 1 and multiply by 3
      int setSpeed = (data - '0' + 1) * 5;

      // Check if the calculated speed is within acceptable range
      if (setSpeed > MinSpeedCmS && setSpeed <= MaxSpeedCmS)
      {

        // Update the target speed and the almost car speed with the new value

        targetSpeedCmS = setSpeed;
        carSpeedAlmostCmS = targetSpeedCmS;

        // Print a message debugging the change in speed (commented out)
        // Serial.println("Inside Mode 2, Changing speed to: " + String(targetSpeedCmS));
      }
    }

    break;
  }
}

/**
 * Sends a message in CSV format.
 *
 * This function is responsible for sending a message in Comma-Separated Values (CSV) format.
 * It performs the necessary operations to convert the message into a CSV string and sends it
 * to the desired destination.
 *
 * The message includes the distance travelled by the car, car speed, nearest obstacle distance,
 * current mode, and other relevant data.
 *
 * The message is constructed using the following format:
 * "Distance travelled: <distance> Speed: <speed> Object at: <obstacle distance> Current mode: <mode>, etc"
 *
 * @param None
 * @return None
 */
void sendMessageCSV()
{

  // Construct a comma-separated value (CSV) message containing various data
  // The data includes: distance travelled by the car in centimeters, average speed in millimeters per second,
  // and distance to the nearest obstacle in some unit (possibly centimeters).
  messageCSV = String(int(distanceTravelledByTheCarCm)) + "," + String(int(experimentalSpeedCmS)) + "," + String(int(nearestObstacleDistance)) + "," + String(int(2 * carSpeedAlmostCmS)) + "," + String(int(currentMode)) + "," + String(int(huskySaw)) + " \n ";

  // Write the constructed CSV message to the Processing client
  // The WebClient object is assumed to have a write function that accepts a character array and its length
  WebClient.write(messageCSV.c_str(), messageCSV.length());

  // Print the constructed CSV message (commented out)
  // Serial.println(messageCSV);
}

/**
 * Sends a message.
 * Unlike the sendMessageCSV function, this function sends a message in a different format.
 * Unused in the current implementation.
 */
void sendMessage()
{
  /*
    message = "Distance travelled: " + String(int(distanceTravelledByTheCar))
              + " Speed: " + String(int(    carSpeedAlmostCmS)
              + ((nearestObstacleDistance != 100) ? " Object at: " + String(int(nearestObstacleDistance)) : " No Object")
              + " Current mode: " + ((currentMode == MODE_1_Object_Following) ? "Mode1 \n" : "Mode2URL \n");

    */

  /*
    message = "Distance travelled: " + String(int((leftPulseCount + rightPulseCount) * 3.142 * radiusOfWheel / EncoderPulsesPerRevolution))
              + " error: " + String(Kp * error)
              + " integral: " + String(Ki * integral)
              + " differential: " + String(Kd * differential)
              + " elapsedTime: " + String(elapsedTime)
              + " Speed: " + String(int(    carSpeedAlmostCmS)
              + ((nearestObstacleDistance != 100) ? " Object at: " + String(int(nearestObstacleDistance)) : " No Object")
              + " Current mode: " + ((currentMode == MODE_1_Object_Following) ? "Mode1 \n" : "Mode2URL \n");
    */

  /*
    message[0] = int(distanceTravelledByTheCar);
    message[1] = int(    carSpeedAlmostCmS;
    message[2] = int(nearestObstacleDistance);



    WebClient.write(message, sizeof(message));
    */
}

// Matrix Handling

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

/**
 * Displays a the above patterns on the LED matrix.
 * The smiley face pattern is loaded onto the matrix using the loadFrame() function.
 */

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
/**
 * Calculates the distance to the closest obstacle using sonar.
 *
 * @return The distance to the closest obstacle in meters.
 */
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
/**
 * @brief Checks the position of the object relative to the current position.
 *
 * This function is responsible for determining the position of the object relative to the current position.
 * It performs the necessary calculations and updates the necessary variables.
 *
 * @return void
 */
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

    currentMode = setMode;
  }

  // Serial.println("Inside Object Tracking, changing speed to: " + String(carSpeedAlmostCmS));
}

// Keeping the car moving while checking infrared sensors for tracking
/**
 * Keeps the vehicle moving while continuously checking the IR sensors.
 * This function is responsible for maintaining the movement of the vehicle
 * while constantly monitoring the IR sensors for any obstacles.
 */
void keepMovingCheckingIRSensors()
{

  // Serial.println("got inside keepMovingCheckingIRSensors");

  int irLeftValue = digitalRead(LeftIRSensorInput);
  int irRightValue = digitalRead(RightIRSensorInput);

  // Move straight for debugging
  // int irLeftValue = LOW;
  // int irRightValue = LOW;

  // Serial.print("irLeftValue: ");
  // Serial.println(irLeftValue);
  // Serial.print("irRightValue: ");
  // Serial.println(irRightValue);

  if (irLeftValue == HIGH && irRightValue == HIGH)
  {

    // Serial.println("Both sensors on track - moving forward");

    // If both sensors are on track, move forward

    moveForwardatSpeed(carSpeedAlmostCmS);
  }
  else if (irLeftValue == LOW && irRightValue == HIGH)
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
  else if (irLeftValue == LOW && irRightValue == LOW)
  {
    if (leftIRSensorSwitchedOnByLens)
    {
      turnLeft();
    }
    else if (rightIRSensorSwitchedOnByLens)
    {
      turnRight();
    }
  }

  // Serial.println("end of keepMovingCheckingIRSensors");
}

// Functions needed for Movement

inline int mapSpeedCmSToPWM(double speed);

// Mapping speed to PWM values
/**
 * Maps the given speed in centimeters per second to a corresponding PWM value.
 *
 * @param speed The speed in centimeters per second.
 * @return The mapped PWM value.
 */
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
/**
 * Moves the vehicle forward at the specified speed.
 *
 * @param speed The speed at which the vehicle should move forward.
 */
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
/**
 * Stops the car.
 */
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
/**
 * Function: turnLeft
 * ------------------
 * This function is responsible for turning the autonomous vehicle to the left.
 * It is used in the Arduino code for the Gold Challenge of the Autonomous Vehicle Arduino Project.
 */
void turnLeft()
{

  // Serial.println("Inside turnLeft - Turning left");

  // Adjust the right motor PWM for a left turn

  // Using direct PWM instead of mapped speeds as the car doesnt
  // turn at too low speeds or skids at too high

  analogWrite(
      RightMotorPWM,
      RightWheelCoefficient * TurnSpeedOuterPulseWidth);

  // Stop the left motor
  analogWrite(
      LeftMotorPWM,
      LeftWheelCoefficient * TurnSpeedInnerPulseWidth);

  // Switch on

  digitalWrite(RightMotorSwitchActive, HIGH);

  digitalWrite(LeftMotorSwitchActive, HIGH);
}

// Function to turn the car to the right
/**
 * Function: turnRight
 * -------------------
 * This function is responsible for turning the autonomous vehicle to the right.
 * It is used in the Arduino code for the Gold Challenge.
 */
void turnRight()
{

  // Serial.println("Inside turnRight - Turning right");

  // Stop the right motor by setting its PWM

  // Using direct PWM instead of mapped speeds as the car doesnt
  // turn at too low speeds or skids at too high

  analogWrite(
      RightMotorPWM,
      RightWheelCoefficient * TurnSpeedInnerPulseWidth);

  // Adjust the left motor PWM for a right turn
  analogWrite(
      LeftMotorPWM,
      LeftWheelCoefficient * TurnSpeedOuterPulseWidth);

  // Switch on

  digitalWrite(RightMotorSwitchActive, HIGH);

  digitalWrite(LeftMotorSwitchActive, HIGH);
}

// decideTheCarsStatus

/**
 * @brief This function decides the status of the car based on the loop counter.
 *        It performs different actions depending on the value of the loop counter.
 *        If the loop counter is a multiple of 23, it checks the position relative to an object and adjusts the car speed accordingly.
 *        If the loop counter is a multiple of 31, it checks the server.
 *        If the loop counter is a multiple of 53, it asks the Husky.
 *        If the loop counter is a multiple of 700, it calculates the distance travelled by the car, calculates the speed, sends a message in CSV format, and resets the loop counter.
 */
void decideTheCarsStatus()
{

  if (loopCounter % 23 == 0)
  {

    // Serial.println("Got into the 23rd loop");

    checkPositionRelativeToObject();

    if (currentMode == MODE_1_Object_Following)
    {

      carSpeedAlmostCmS = PIDObjectFollowing_f_1();

      // Serial.println("Inside Object Tracking, changing speed to: " + String(carSpeedAlmostCmS));
    }
    else if (currentMode == MODE_2_Speed_Control)
    {

      carSpeedAlmostCmS = PIDMaintainSpeed_sc_2();

      // Serial.println("Inside Speed PID change, changing speed to: " + String(carSpeedAlmostCmS));
    }
  }
  else if (loopCounter % 31 == 0)
  {

    checkServer();
  }
  else if (loopCounter % 53 == 0)
  {

    askHusky();
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
}

// moveITmaybe

/**
 * 
 * This function is responsible for moving the vehicle based on certain conditions.
 * The Conditions include:
 * 1. If no obstacles are too close, and the vehicle is not stopped through the GUI or the lens, 
 * it continues moving and checking the infrared sensors.
 * 
 * 2. If an obstacle is too close, and the vehicle is not stopped through the GUI or the lens,
 *  the vehicle delays for a brief period and then checks the position relative to the object.
 * 
 * 3. If the vehicle is stopped through the GUI or the lens, it doesnt do anything.
 * 
 */
void moveITmaybe()
{

  if (!obstacleTooClose && !StopTheCarThroughGUI && !StopTheCarThroughLens)
  {

    // Continue moving and checking infrared sensors if no obstacles are too close
    // Serial.print("MOVE BRO?");  // Optional debugging statement

    keepMovingCheckingIRSensors();
  }
  else if (!StopTheCarThroughGUI && !StopTheCarThroughLens) // obstacleTooClose &&
  {

    delayMicroseconds(3000);

    checkPositionRelativeToObject();
  }
}

// Initialise stuff

/**
 * @brief Initializes the necessary stuff.
 *
 * This function is responsible for initializing the necessary components or variables
 * required for the program to run correctly.
 *
 * @return void
 */
void initialiseStuff()
{

  pinMode(LeftIRSensorInput, INPUT);  // Configure the left infrared sensor pin as an input
  pinMode(RightIRSensorInput, INPUT); // Configure the right infrared sensor pin as an input

  pinMode(UltrasonicTrigger, OUTPUT);     // Configure the ultrasonic trigger pin as an output
  pinMode(UltrasonicEchoDetector, INPUT); // Configure the ultrasonic echo detector pin as an input

  pinMode(LeftMotorSwitch3, OUTPUT); // Configure the left motor control pin 3 as an output
  pinMode(LeftMotorSwitch4, OUTPUT); // Configure the left motor control pin 4 as an output

  pinMode(RightMotorSwitch1, OUTPUT); // Configure the right motor control pin 1 as an output
  pinMode(RightMotorSwitch2, OUTPUT); // Configure the right motor control pin 2 as an output

  digitalWrite(RightMotorSwitch1, LOW); // Initially set both motors to be swtiched off
  digitalWrite(RightMotorSwitch2, LOW); // Initially set both motors to be swtiched off

  digitalWrite(LeftMotorSwitch3, LOW); // Initially set both motors to be swtiched off
  digitalWrite(LeftMotorSwitch4, LOW); // Initially set both motors to be swtiched off

  pinMode(LeftMotorPWM, OUTPUT);  // Configure the left motor PWM pin as an output
  pinMode(RightMotorPWM, OUTPUT); // Configure the right motor PWM pin as an output

  pinMode(LeftEncoder, INPUT_PULLUP);  // Configure the left motor encoder pin with pull-up resistor enabled
  pinMode(RightEncoder, INPUT_PULLUP); // Configure the right motor encoder pin with pull-up resistor enabled
                                       // & logic inversion (a 20 k resistor in parallel for impedence control)

  pinMode(SPEAKER_PIN, OUTPUT); // Set the speaker pin as an output

  // Initialize the LED matrix for further use
  matrix.begin();
}

// Arduino functions

// Initialization function executed once at the start of the program
/**
 * @brief Initializes the Arduino board and sets up the necessary components and connections.
 *
 * This function is called once when the Arduino board is powered on or reset. It initializes the serial communication,
 * sets up the connection with the client, configures the interrupt service routines (ISRs) for the encoders, and performs
 * other necessary setup tasks. After the setup is completed, the Arduino is ready to execute the main loop.
 */
void setup()
{

  Serial.begin(115200); // Initialize Serial communication with a baud rate of 115200

  Serial.println(".");
  Serial.println(".");
  Serial.println(".");
  Serial.println(".");
  Serial.println(".");

  Serial.println("Starting to setup Car");

  // initialise everything

  initialiseStuff();

  matrix.loadFrame(LEDMATRIX_BOOTLOADER_ON);

  delay(5000);

  // Setup Connection

  Serial.println("Setting up!");

  Serial.println("Inside Setup before Connection Setup!");

  connectionSetup();

  Serial.println("Inside Setup afer Connection Setup! WOW!");

  // Interrupts

  // Serial.println("1 Inside Setup 2");

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

  // Serial.println(" 4 Inside Setup 3");

  // This line initiates a while loop that continues until a connection is established with the client.
  // It checks if the client is not connected.

  // matrix.loadWrapper(LEDMATRIX_ANIMATION_WIFI_SEARCH, sizeof(LEDMATRIX_ANIMATION_WIFI_SEARCH) / sizeof(LEDMATRIX_ANIMATION_WIFI_SEARCH[0]));
  // matrix.play(false);

  matrix.loadFrame(LEDMATRIX_EMOJI_SAD);

  delay(5000);

  Serial.println("Starting to look for the the Client");

  connectClient();

  Serial.println("Connected to Client!");
  Serial.println("Connected to Client!");
  Serial.println("Connected to Client!");

  Serial.println("Setting up Husky Lens!");

  huskyLensSetup();

  Serial.println("Husky Lens is Setup!");

  matrix.loadFrame(LEDMATRIX_LIKE);

  delay(5000);

  Serial.println("Setup Completed!");

  // matrix.endDraw();

  // matrix.loadWrapper(LEDMATRIX_ANIMATION_LOCK, sizeof(LEDMATRIX_ANIMATION_LOCK) / sizeof(LEDMATRIX_ANIMATION_LOCK[0]));
  // matrix.play();

  matrix.loadFrame(LEDMATRIX_EMOJI_SAD);

  Serial.println("Please Login!");

  huskyLensLogin();

  matrix.loadFrame(LEDMATRIX_EMOJI_HAPPY);

  Serial.println("Let's start loop!");

  Serial.println("Happy Driving!");

  noTone(SPEAKER_PIN); // Stop the tone
}

// Main execution loop function that runs continuously after setup
/**
 * @brief The main loop function that runs repeatedly in the Arduino program.
 *
 * This function is responsible for controlling the primary logic of the program.
 * It delegates the control to other functions, such as `decideTheCarsStatus()` and `moveITmaybe()`.
 * It also increments the loop counter for each iteration.
 */

void loop()
{
  // Delegate the primary control logic
  // Serial.print('.');

  // Serial.println("starting loop");  // Optional debugging statement

  decideTheCarsStatus();

  // Serial.println("Is the obstacleTooClose?");  // Optional debugging statement
  // Serial.println(obstacleTooClose);            // Optional debugging statement

  moveITmaybe();

  // Increment the loop counter for each iteration
  loopCounter++;

  // Serial.println("end of loop");
}

