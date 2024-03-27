#include <pidautotuner.h>

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
bool StopTheCarThroughGUI = false;
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

// Start them all at 1, 0, zero and then autotune them
double Kp_f_1 = 1;
double Ki_f_1 = 0;
double Kd_f_1 = 0;

double error_f_1 = 0;
double previousError_f_1 = 0;
double integral_f_1 = 0;
double differential_f_1 = 0;
double controlSignal_f_1 = 0;
double currentTime_f_1 = infinity();
double previousTime_f_1 = millis();
double elapsedTime_f_1 = 0;

double PIDObjectFollowing_f_1()
{
  currentTime_f_1 = millis();
  elapsedTime_f_1 = (double)(currentTime_f_1 - previousTime_f_1);
  error_f_1 = (double)(nearestObstacleDistance - ObjectFollowingDistance);
  integral_f_1 += (error_f_1 < 10) ? (double)(error_f_1 * elapsedTime_f_1) : 0;
  differential_f_1 = (double)((error_f_1 - previousError_f_1) / elapsedTime_f_1);
  controlSignal_f_1 = (double)(Kp_f_1 * error_f_1 + Ki_f_1 * integral_f_1 + Kd_f_1 * differential_f_1);
  previousError_f_1 = error_f_1;
  previousTime_f_1 = currentTime_f_1;
  double nextSpeed_f_1 = carSpeedAlmostCmS + controlSignal_f_1;
  nextSpeed_f_1 = constrain(nextSpeed_f_1, MinSpeedCmS, MaxSpeedCmS);
  return nextSpeed_f_1;
}

// Autotuning constants
double Ku = 0.6 * Kp_f_1;
double Tu = millis();
double Ki_auto()
{
  return 0.45 * Ku / Tu;
};
double Kd_auto()
{
  return 0.125 * Ku * Tu;
};

double currentTime_u = infinity();
double previousTime_u = millis();

double maxSoFar = MinSpeedCmS;
double minSoFar = MaxSpeedCmS;

// Autotune PID function
void autotunePID()
{

  maxSoFar = (maxSoFar > carSpeedAlmostCmS) ? maxSoFar : carSpeedAlmostCmS;
  minSoFar = (minSoFar < carSpeedAlmostCmS) ? minSoFar : carSpeedAlmostCmS;

  currentTime_u = millis();
  Tu = (double)(currentTime_u - previousTime_u);

  Ku = (Ku * ((MaxSpeedCmS - MinSpeedCmS))) / (maxSoFar - minSoFar);

  Kp_f_1 = Ku;
  Ki_f_1 = Ki_auto();
  Kd_f_1 = Kd_auto();

  previousTime_u = currentTime_u;
}

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

  /*

  PIDAutotuner tuner = PIDAutotuner();

  // Set the target value to tune to
  // This will depend on what you are tuning. This should be set to a value within
  // the usual range of the setpoint. For low-inertia systems, values at the lower
  // end of this range usually give better results. For anything else, start with a
  // value at the middle of the range.
  tuner.setTargetInputValue(130);

  // Set the loop interval in microseconds
  // This must be the same as the interval the PID control loop will run at
  tuner.setLoopInterval(10000);

  // Set the output range
  // These are the minimum and maximum possible output values of whatever you are
  // using to control the system (Arduino analogWrite, for example, is 0-255)
  tuner.setOutputRange(0, 255);

  // Set the Ziegler-Nichols tuning mode
  // Set it to either PIDAutotuner::ZNModeBasicPID, PIDAutotuner::ZNModeLessOvershoot,
  // or PIDAutotuner::ZNModeNoOvershoot. Defaults to ZNModeNoOvershoot as it is the
  // safest option.
  tuner.setZNMode(PIDAutotuner::ZNModeBasicPID);

  // This must be called immediately before the tuning loop
  // Must be called with the current time in microseconds
  tuner.startTuningLoop(micros());

  // Run a loop until tuner.isFinished() returns true
  long microseconds;

  while (!tuner.isFinished()) {

    // This loop must run at the same speed as the PID control loop being tuned
    long prevMicroseconds = microseconds;
    microseconds = micros();

    // Get input value here (temperature, encoder position, velocity, etc)
    double input = doSomethingToGetInput();

    // Call tunePID() with the input value and current time in microseconds
    double output = tuner.tunePID(input, microseconds);

    // Set the output - tunePid() will return values within the range configured
    // by setOutputRange(). Don't change the value or the tuning results will be
    // incorrect.
    moveForwardatSpeed(output);

    // This loop must run at the same speed as the PID control loop being tuned
    while (micros() - microseconds < loopInterval) delayMicroseconds(1);
  }

  // Turn the output off here.
  moveForwardatSpeed(0);

  // Get PID gains - set your PID controller's gains to these
  Kp_f_1 = tuner.getKp();
  Ki_f_1 = tuner.getKi();
  Kd_f_1 = tuner.getKd();

  */
}

void loop()
{
  if (loopCounter % 23 == 0)
  {

    checkPositionRelativeToObject();

    carSpeedAlmostCmS = PIDObjectFollowing_f_1();
  }
  if (loopCounter % 37 == 0)
  {

    autotunePID();
  }
  else if (loopCounter % 700 == 0)
  {

    Serial.println("Kp : " + String(Kp_f_1) + " Ki : " + String(Ki_f_1) + " Kd : " + String(Kd_f_1));

    loopCounter = 0;
    integral_f_1 /= 50;
  }

  moveForwardatSpeed(carSpeedAlmostCmS);

  loopCounter++;
}
