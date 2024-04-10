import processing.net.*;
import controlP5.*;
import meter.*;
import processing.sound.*;
import gifAnimation.*;

Gif gif;

SoundFile file;
boolean played = false;

// Sound play;

String serverAddress = "192.168.4.1";
Client myClient;

ControlP5 cp5;
Slider slide;
Meter speedometer;

char startURL = 'Z';
char stopURL = 'X';
char displayHeartURL = 'H';
char displaySmileyURL = 'S';
char displayW5URL = 'W';
char mode0URL = 'A';
char mode2URL = 'C';
char turnAroundURL = 'T';
char FreeBirdURL = 'F';

boolean StopStart = false;
boolean SwitchModes = false;

String Distance_travelled = "0";
String SpeedTravelled = "0";
String SpeedPWMpercent = "0";
String ObstacleDistance = "5";
String ModeReadFromServer = "0";
String TagReadFromServer = "0";
String lastTagSeen = "0";
String leftIRSensorSwitchedOnByLens = "0";
String rightIRSensorSwitchedOnByLens = "0";
String ModeName = "Object Following";

String[] values;
String message = "0,0,0,0,0,0";

// Setup function called once at the beginning
void setup(){
  
  myClient = new Client(this, serverAddress, 5200);

  size(1200, 1100);

  gif = new Gif(this, "freedom.gif");
  gif.play();

  cp5 = new ControlP5(this);

  int buttonX = 100;
  int buttonY = 50;
  int buttonWidth = 300;
  int buttonHeight = 100;
  int buttonSpacing = 140;

  cp5.addToggle("StopStart")
    .setPosition(buttonX, buttonY)
    .setSize(buttonWidth, buttonHeight)
    .setValue(false)
    .setMode(ControlP5.SWITCH)
    .setColorActive(color(100, 100, 100))
    .setCaptionLabel("OFF/ON")
    .getCaptionLabel().setFont(createFont("Arial", 30));

  cp5.addToggle("SwitchModes")
    .setPosition(buttonX, buttonY + buttonSpacing)
    .setSize(100, 20)
    .setValue(false)
    .setSize(buttonWidth, buttonHeight)
    .setMode(ControlP5.SWITCH)
    .setColorActive(color(100, 100, 100))
    .setCaptionLabel("Mode0/Mode2")
    .getCaptionLabel().setFont(createFont("Arial", 30));

  cp5.addButton("displayHeart")
    .setPosition(buttonX, buttonY + 2 * buttonSpacing)
    .setSize(buttonWidth, buttonHeight)
    .setLabel("Display Heart")
    .getCaptionLabel().setFont(createFont("Arial", 30));

  cp5.addButton("displaySmiley")
    .setPosition(buttonX, buttonY + 3 * buttonSpacing)
    .setSize(buttonWidth, buttonHeight)
    .setLabel("Display Smiley")
    .getCaptionLabel().setFont(createFont("Arial", 30));

  cp5.addButton("displayW5")
    .setPosition(buttonX, buttonY + 4 *  buttonSpacing)
    .setSize(buttonWidth, buttonHeight)
    .setLabel("Display W5")
    .getCaptionLabel().setFont(createFont("Arial", 46));

  cp5.addButton("TurnAround")
    .setPosition(buttonX, buttonY + 5 *  buttonSpacing)
    .setSize(buttonWidth, buttonHeight)
    .setLabel("Turn Around")
    .getCaptionLabel().setFont(createFont("Arial", 30));

  cp5.addButton("FreeBird")
    .setPosition(buttonX, buttonY + 6 *  buttonSpacing)
    .setSize(buttonWidth, buttonHeight)
    .setLabel("Free Bird")
    .getCaptionLabel().setFont(createFont("Arial", 30));

  slide = cp5.addSlider("SpeedControl")
    .setRange(1, 10)
    .setValue(10)
    .setPosition(buttonX - 35, buttonY + 7 * buttonSpacing)
    .setSize(370, 50)
    .setNumberOfTickMarks(10)
    .setSliderMode(Slider.FIX)
    .snapToTickMarks(true)
    .setLabel("Speed");

  slide.getValueLabel().setFont(createFont("Arial", 50));
  slide.getCaptionLabel().setFont(createFont("Arial", 30)).align(ControlP5.CENTER, ControlP5.BOTTOM_OUTSIDE);

  speedometer = new Meter(this, 500, 380, false);
  speedometer.setMeterWidth(600);
  speedometer.setUp(0, 100, 0, 100, 180, 360);
  speedometer.setMinScaleValue(0.0);
  speedometer.setMaxScaleValue(100.0);
  speedometer.setTitle("Speed");

  String[] scaleLabels = {"0", "25", "50", "75", "100"};
  speedometer.setScaleLabels(scaleLabels);
}




void draw() {

  background(0);
  String valueLabel = String.valueOf((int)(slide.getValue() * 5));
  slide.getValueLabel().setText(valueLabel);

  readClientCSV();

  textSize(32);
  text("Distance travelled: " + Distance_travelled, 500, 100);
  text("Obstacle distance: " + ((ObstacleDistance.charAt(0) == '5')? "No Object within 50 cm" : ObstacleDistance), 500, 200);

  if (ModeReadFromServer.charAt(0)== '0')
  {
    ModeName = "Tag Controlled Speed";
  } else if  (ModeReadFromServer.charAt(0)== '1')
  {
    ModeName = "Object Following";
  } else if  (ModeReadFromServer.charAt(0)== '2')
  {
    ModeName = "GUI Controlled Target Speed";
  }

  text("Mode: " + ModeName, 500, 300);

  text("Speed percentage of PWM Signal: " + SpeedPWMpercent, 500, 800);

  text("Tag read: " + TagReadFromServer, 500, 850);
  text("Tag Speed Control read: " + lastTagSeen, 500, 900);
  text("leftIRSensorSwitchedOnByLens: " + leftIRSensorSwitchedOnByLens, 500, 950);
  text("rightIRSensorSwitchedOnByLens: " + rightIRSensorSwitchedOnByLens, 500, 1000);


  speedometer.updateMeter(int(SpeedTravelled));
}

void StopStart(boolean theFlag) 
{
  if (theFlag) {

    sendCommand(startURL);

    cp5.setColorBackground(color(0, 255, 0));

    if (file != null) {
      file.stop();
    }

    // file.pause();

    file = new SoundFile(this, "ACDC.mp3");
    file.loop();
  } else {

    sendCommand(stopURL);
    cp5.setColorBackground(color(255, 0, 0));

    if (file != null) {
      file.stop();
    }

    // file.pause();
  }
}

void SwitchModes(boolean theFlag) {
  if (theFlag) {

    sendCommand(mode2URL);

    if (file != null) {
      file.stop();
    }
    // file.pause();

    file = new SoundFile(this, "Run.mp3");
    file.loop();

    played = true;
  } else {

    sendCommand(mode0URL);

    if (file != null) {
      file.stop();
    }
    // file.pause();

    if (played) {
      file = new SoundFile(this, "Eye.mp3");
      file.loop();
    }
  }
}

void displayHeart() {
  sendCommand(displayHeartURL);

  if (file != null) {
    file.stop();
  }

  // file.pause();

  file = new SoundFile(this, "Love.mp3");
  file.loop();
}

void TurnAround() {
  sendCommand(turnAroundURL);

  if (file != null) {
    file.stop();
  }
  // file.pause();

  file = new SoundFile(this, "Turn.mp3");
  file.loop();
}

void FreeBird() {
  sendCommand(FreeBirdURL);

  if (file != null) {
    file.stop();
  }
  // file.pause();

  file = new SoundFile(this, "Free.mp3");
  file.loop();

  image(gif, 0, 0);
}

void displaySmiley() {

  sendCommand(displaySmileyURL);

  if (file != null) {
    file.stop();
  }

  // file.pause();

  file = new SoundFile(this, "Happy.mp3");
  file.loop();
}

void displayW5() {
  sendCommand(displayW5URL);

  if (file != null) {
    file.stop();
  }

  // file.pause();

  file = new SoundFile(this, "Sweet.mp3");
  file.loop();
}

void SpeedControl(int speed) {
  sendCommand(char((char) ((speed - 1) + '0')));
}

void sendCommand(char command) {
  if (myClient != null) {
    myClient.write(command);
  }
}

void readClientCSV() {
  message = myClient.readStringUntil('\n');
  if (message != null) {
    values = split(message, ',');
    if (values.length == 9) {
      Distance_travelled = values[0];
      SpeedTravelled = values[1];
      ObstacleDistance = values[2];
      SpeedPWMpercent = values[3];
      ModeReadFromServer = values[4];
      TagReadFromServer = values[5];
      lastTagSeen = values[6];
      leftIRSensorSwitchedOnByLens = values[7];
      rightIRSensorSwitchedOnByLens = values[8];
    }
  }
}
