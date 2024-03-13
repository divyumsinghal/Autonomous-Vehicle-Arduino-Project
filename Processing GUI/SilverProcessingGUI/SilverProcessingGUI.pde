// Import the ControlP5 and processing.net libraries
import controlP5.*;
import processing.net.*;

// Arduino's IP address (replace with actual IP address)
String serverAddress = "192.168.0.110";

// Declare ControlP5 object
ControlP5 cp5;

// Declare a Client object for network communication
Client myClient;

// URLs for different car commands
char startURL = 'Z';
char stopURL = 'X';
char displayHeartURL = 'H';
char displaySmileyURL = 'S';
char displayW5URL = 'W';
char mode1URL = 'A';
char mode2URL = 'B';

// booleans for switches
boolean StopStart = false;
boolean SwitchModes = false;

// Message variables
int Distance_travelled = 0;
int Speed = 0;
int Obstacle = 0;
boolean Mode = false;


// Setup function called once at the beginning
void setup()
{

  // Initialize client for communication with Arduino
  myClient = new Client(this, serverAddress, 5200);

  // Set the size of the Processing window
  size(900, 900);

  // Initialize controlP5
  cp5 = new ControlP5(this);

  // Positioning variables for buttons
  int buttonX = 100;
  int buttonY = 50;
  int buttonWidth = 300;
  int buttonHeight = 100;
  int buttonSpacing = 140;

  // Add a switches to the GUI

  cp5.addToggle("StopStart")
    .setPosition(buttonX, buttonY)
    .setSize(100, 20)
    .setValue(false)
    .setSize(buttonWidth, buttonHeight)
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
    .setCaptionLabel("Mode1/Mode2")
    .getCaptionLabel().setFont(createFont("Arial", 30));


  // Add buttons to the GUI for controlling LED matrix display
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


  // Create a slider with values from 1 to 10
  Slider slide = cp5.addSlider("SpeedControl")
    .setRange(1, 10)
    .setValue(10)
    .setPosition(buttonX - 35, buttonY + 5 * buttonSpacing)
    .setSize(370, 50)
    .setNumberOfTickMarks(10)
    .setSliderMode(Slider.FIX)
    .snapToTickMarks(true)
    .setLabel("Speed");  // Set font directly on the slider

  slide.getValueLabel().setFont(createFont("Arial", 50));
  slide.getCaptionLabel().setFont(createFont("Arial", 30)).align(ControlP5.CENTER, ControlP5.BOTTOM_OUTSIDE);
}


// Function to send commands to the Arduino
void sendCommand(char command)
{

  // Load the URL to send the command to the Arduino
  if (myClient != null)
  {
    myClient.write(command);
    //println("Sent Command");
    //println("Sending command: " + command);
    //println(myClient.ip());
  } else
  {
    println("Shut up");
  }
}

void StopStart(boolean theFlag)
{
  if (theFlag)
  {
    sendCommand(startURL);
    cp5.setColorBackground(color(0, 255, 0));
  } else
  {
    sendCommand(stopURL);
    cp5.setColorBackground(color(255, 0, 0));
  }
}

void SwitchModes(boolean theFlag)
{
  if (theFlag)
  {
    sendCommand(mode2URL);
    //cp5.setColorBackground(color(0,255,0));
  } else
  {
    sendCommand(mode1URL);
    //cp5.setColorBackground(color(255,0,0));
  }
  Mode = theFlag;
}

// Display a heart on the LED matrix
public void displayHeart()
{
  sendCommand(displayHeartURL);
}

// Display a smiley on the LED matrix
public void displaySmiley()
{
  sendCommand(displaySmileyURL);
}

// Display a pattern on the LED matrix
public void displayW5()
{
  sendCommand(displayW5URL);
}

void SpeedControl(int speed) {
  sendCommand(char((char) ((speed - 1) + '0')));
}

void readClientArr()
{
  byte[] data = new byte[3]; // Assuming your data array has 3 elements

  // Read data from the client
  int bytesRead = myClient.readBytes(data);

  if (bytesRead == data.length) {
    // Successfully read the expected number of bytes

    // Now you can interpret and print the data

    Distance_travelled = data[0];
    Speed = data[1];
    Obstacle = data[2];

    println(
      "Distance travelled: " + data[0] +
      " Speed: " + data[1] +
      " Obstacle: " + data[2] +
      " Mode: " + ((Mode) ? "mode 2" : "mode 1")
      );
  } else {
    // Handle the case where not enough bytes were read
    println("Error reading data from the client");
  }
}

void readClientCSV()
{
  // Read a line of text from the client
  String message = myClient.readStringUntil('\n');

  if (message != null)
  {
    // Split the message into individual values
    String[] values = split(message, ',');

    if (values.length == 3)
    {
      // Successfully split the message into three values

      Distance_travelled = Integer.parseInt(values[0]);
      Speed = Integer.parseInt(values[1]);
      Obstacle = Integer.parseInt(values[2]);

      // Now you can interpret and print the values
      println
        (
        "Control Signal: " + Distance_travelled +
        " Speed: " + Speed +
        " Obstacle: " + Obstacle +
        " Mode: " + ((Mode) ? "mode 2" : "mode 1")
        );
    }
  };
}


// Draw function runs continuously
void draw()
{
  background(0);

  /*
  textSize(32);
   text("Distance travelled: " + Distance_travelled, 500, 100);
   text("Obstacle distance: " + Obstacle, 500, 200);
   text("Mode: " + (Mode ? "2" : "1"), 500, 300);
   */

  // Read data from the Arduino
  readClientCSV();

  textSize(32);
  text("Distance travelled: " + Distance_travelled, 500, 100);
  text("Obstacle distance:" + Obstacle, 500, 200);
  text("Speed: " + Speed, 500, 300);
  text("Mode: " + (Mode ? "2" : "1"), 500, 400);
};
