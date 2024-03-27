#include <AutoPID.h>

const int LeftIRSensorInput = A1;
const int RightIRSensorInput = A0;
const int RightMotorPWM = 10;
const int LeftMotorPWM = 9;
const int RightMotorSwitch1 = 11;
const int RightMotorSwitch2 = 12;
const int LeftMotorSwitch3 = 8;
const int LeftMotorSwitch4 = 7;
const int UltrasonicTrigger = 5;
const int UltrasonicEchoDetector = 4;
const int LED_PIN = 13;
int RightMotorSwitchActive = RightMotorSwitch1;
int LeftMotorSwitchActive = LeftMotorSwitch3;

const int RightEncoder = 3;
const int LeftEncoder = 2;
volatile unsigned long leftPulseCount = 0;
volatile unsigned long rightPulseCount = 0;

const double RightWheelCoefficient = 1;
const double LeftWheelCoefficient = 0.59;
const double MinSpeedCmS = 0;
const double MaxSpeedCmS = 50;
const int PWMMin = 0;
const int PWMMax = 225;
const int turnSpeedOuterPulseWidth = 190;
const int turnSpeedInnerPulseWidth = 60;
const int EncoderPulsesPerRevolution = 4;
const int CriticalObjectDistance = 10;
const int ObjectFollowingDistance = 20;
const int Overtime = 51;
const double SPEED_OF_SOUND_CM_PER_MS = 0.017;
const double radiusOfWheelCm = 3.5;
const double radiusOfWheel = 35;

double nearestObstacleDistance = 100;
bool obstacleTooClose = false;
double carSpeedAlmostCmS = MaxSpeedCmS;
unsigned long loopCounter = 0;
bool StopTheCarThroughGUI = true;
double distanceTravelledByTheCarCm = (leftPulseCount + rightPulseCount) * 3.142 * radiusOfWheelCm / EncoderPulsesPerRevolution;
double targetSpeedCmS_MODE_2_Speed_Control_PID = MaxSpeedCmS;

double arcLengthMm = (double)(1000 * radiusOfWheel * 45 / 360);
volatile unsigned long leftTimePrev = millis();
volatile unsigned long leftTimeCurrent = infinity();
volatile unsigned long rightTimePrev = millis();
volatile unsigned long rightTimeCurrent = infinity();
volatile unsigned long leftTimeCurrentPast = leftTimeCurrent;
volatile unsigned long rightTimeCurrentPast = rightTimeCurrent;
double leftSpeedCmS = (double)(arcLengthMm / (leftTimeCurrent - leftTimePrev));
double rightSpeedCmS = (double)(arcLengthMm / (rightTimeCurrent - rightTimePrev));
double averageSpeedCmS = (double)((leftSpeedCmS + rightSpeedCmS) / 2);

// PID

// PID for Speed Control

// Define PID constants

double Kp_sc_2 = 0.005;      // Proportional gain
long double Ki_sc_2 = 5e-12; // Integral gain
double Kd_sc_2 = 2e-12;      // Derivative gain

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
  error_sc_2 = (double)(targetSpeedCmS_MODE_2_Speed_Control_PID - averageSpeedCmS);

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
  // Serial.println("averageSpeedCmS = " + String(averageSpeedCmS));
  // Serial.println("carSpeedAlmostCmS = " + String(carSpeedAlmostCmS));
  // Serial.println("error_sc_2: " + String(error_sc_2));
  // Serial.println("differential_sc_2: " + String(differential_sc_2 * Kd_sc_2));
  // Serial.println("controlSignal_sc_2: " + String(controlSignal_sc_2));
  // Serial.println("nextSpeed_sc_2: " + String(nextSpeed_sc_2) + "\n");

  return nextSpeed_sc_2;
}

// Autotune

AutoPID myPID(&carSpeedAlmostCmS, &targetSpeedCmS, &carSpeedAlmostCmS, MaxSpeedCmS, MinSpeedCmS, Kp_sc_2, Ki_sc_2, Kd_sc_2);

void calculateSpeed()
{
  rightTimeCurrent = (rightTimeCurrent == rightTimeCurrentPast) ? (9 * rightTimeCurrent + millis()) / 10 : rightTimeCurrent;
  leftTimeCurrent = (leftTimeCurrent == leftTimeCurrentPast) ? (9 * leftTimeCurrent + millis()) / 10 : leftTimeCurrent;
  leftTimeCurrentPast = leftTimeCurrent;
  rightTimeCurrentPast = rightTimeCurrent;
  leftSpeedCmS = (double)(arcLengthMm / (leftTimeCurrent - leftTimePrev));
  rightSpeedCmS = (double)(arcLengthMm / (rightTimeCurrent - rightTimePrev));
  averageSpeedCmS = (double)((leftSpeedCmS + rightSpeedCmS) / 2);
}

double closestObstacleUsingSonar()
{
  digitalWrite(UltrasonicTrigger, LOW);
  delayMicroseconds(2);
  digitalWrite(UltrasonicTrigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(UltrasonicTrigger, LOW);
  unsigned long pulseDuration = pulseIn(UltrasonicEchoDetector, HIGH, 3000);
  return (pulseDuration > 0) ? pulseDuration * SPEED_OF_SOUND_CM_PER_MS : Overtime;
}

void checkPositionRelativeToObject()
{
  nearestObstacleDistance = closestObstacleUsingSonar();
  if (nearestObstacleDistance <= CriticalObjectDistance)
  {
    obstacleTooClose = true;
    stopCar();
    return;
  }
  else
  {
    obstacleTooClose = false;
  }
}

void keepMovingCheckingIRSensors()
{
  int irLeftValue = digitalRead(LeftIRSensorInput);
  int irRightValue = digitalRead(RightIRSensorInput);
  if (irLeftValue == LOW && irRightValue == HIGH)
  {
    turnLeft();
  }
  else if (irLeftValue == HIGH && irRightValue == LOW)
  {
    turnRight();
  }
  else
  {
    moveForwardatSpeed(carSpeedAlmostCmS);
  }
}

inline int mapSpeedCmSToPWM(double speed);

inline int mapSpeedCmSToPWM(double speed)
{
  int PWMValue = round(
      map(speed,
          MinSpeedCmS,
          MaxSpeedCmS,
          PWMMin,
          PWMMax));
  return PWMValue;
}

void moveForwardatSpeed(double speed)
{
  analogWrite(
      RightMotorPWM,
      mapSpeedCmSToPWM(RightWheelCoefficient * speed));
  analogWrite(
      LeftMotorPWM,
      mapSpeedCmSToPWM(LeftWheelCoefficient * speed));
  digitalWrite(RightMotorSwitchActive, HIGH);
  digitalWrite(LeftMotorSwitchActive, HIGH);
}

void stopCar()
{
  analogWrite(
      RightMotorPWM,
      mapSpeedCmSToPWM(0));
  analogWrite(
      LeftMotorPWM,
      mapSpeedCmSToPWM(0));
  digitalWrite(RightMotorSwitchActive, LOW);
  digitalWrite(LeftMotorSwitchActive, LOW);
}

void turnLeft()
{
  analogWrite(
      RightMotorPWM,
      RightWheelCoefficient * turnSpeedOuterPulseWidth);
  analogWrite(
      LeftMotorPWM,
      LeftWheelCoefficient * turnSpeedInnerPulseWidth);
  digitalWrite(RightMotorSwitchActive, HIGH);
  digitalWrite(LeftMotorSwitchActive, HIGH);
}

void turnRight()
{
  analogWrite(
      RightMotorPWM,
      RightWheelCoefficient * turnSpeedInnerPulseWidth);
  analogWrite(
      LeftMotorPWM,
      LeftWheelCoefficient * turnSpeedOuterPulseWidth);
  digitalWrite(RightMotorSwitchActive, HIGH);
  digitalWrite(LeftMotorSwitchActive, HIGH);
}

void setup()
{
  Serial.begin(9600);
  pinMode(LeftIRSensorInput, INPUT);
  pinMode(RightIRSensorInput, INPUT);
  pinMode(UltrasonicTrigger, OUTPUT);
  pinMode(UltrasonicEchoDetector, INPUT);
  pinMode(LeftMotorSwitch3, OUTPUT);
  pinMode(LeftMotorSwitch4, OUTPUT);
  pinMode(RightMotorSwitch1, OUTPUT);
  pinMode(RightMotorSwitch2, OUTPUT);
  digitalWrite(RightMotorSwitch1, LOW);
  digitalWrite(RightMotorSwitch2, LOW);
  digitalWrite(LeftMotorSwitch3, LOW);
  digitalWrite(LeftMotorSwitch4, LOW);
  pinMode(LeftMotorPWM, OUTPUT);
  pinMode(RightMotorPWM, OUTPUT);
  pinMode(LeftEncoder, INPUT_PULLUP);
  pinMode(RightEncoder, INPUT_PULLUP);

  attachInterrupt(
      digitalPinToInterrupt(RightEncoder), []()
      {
      rightPulseCount++;
      rightTimePrev = rightTimeCurrent;
      rightTimeCurrent = millis(); },
      RISING);

  attachInterrupt(
      digitalPinToInterrupt(LeftEncoder), []()
      {
      leftPulseCount++;
      leftTimePrev = leftTimeCurrent;
      leftTimeCurrent = millis(); },
      RISING);

  // if temperature is more than 20 degrees below or above setpoint, OUTPUT will be set to min or max respectively
  myPID.setBangBang(20);
  // set PID update interval to 4000ms
  myPID.setTimeStep(4000);
}

void loop()
{
  if (loopCounter % 23 == 0)
  {

    checkPositionRelativeToObject();

    // carSpeedAlmostCmS = PIDObjectFollowing_sc_2();

    myPID.run(); // call every loop, updates automatically at certain time interval
  }
  if (loopCounter % 37 == 0)
  {

    // autotunePID();
  }
  else if (loopCounter % 700 == 0)
  {

    Serial.print("Kp : ");
    Serial.print(Kp_sc_2);
    Serial.print(" Ki : ");
    Serial.print(Ki_sc_2);
    Serial.print(" Kd : ");
    Serial.println(Kd_sc_2);

    loopCounter = 0;

    integral_sc_2 /= 50;
  }
  if (!obstacleTooClose && !StopTheCarThroughGUI)
  {
    keepMovingCheckingIRSensors();
  }
  else if (!StopTheCarThroughGUI)
  {
    delayMicroseconds(3000);
    checkPositionRelativeToObject();
  }

  loopCounter++;
}
