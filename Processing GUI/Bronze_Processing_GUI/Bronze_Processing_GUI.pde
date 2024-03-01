// Import the ControlP5 and processing.net libraries
import controlP5.*;

// 141 127

// Declare ControlP5 object
ControlP5 cp5;

import processing.net.*;

// Declare a Client object for network communication
Client myClient;

// Arduino's IP address (replace with actual IP address)
String serverAddress = "192.168.4.1";

// String to store received data from the Arduino
String data;

// URLs for different car commands
char startURL = 'Z';
char stopURL = 'X';
char displayHeartURL = 'H';
char displaySmileyURL = 'S';
char displayW5URL = 'W';

boolean StopStart = false;

// Setup function called once at the beginning
void setup()
{
  println("Setting Up");
  
  // Initialize client for communication with Arduino
  myClient = new Client(this, serverAddress, 5200);
  
  println("Connected");

  // Set the size of the Processing window
  size(500, 600);

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
     .setPosition(buttonX,buttonY)
     .setSize(100,20)
     .setValue(false)
     .setSize(buttonWidth, buttonHeight)
     .setMode(ControlP5.SWITCH)
     .setColorActive(color(100,100,100))
     .setCaptionLabel("OFF/ON")
     .getCaptionLabel().setFont(createFont("Arial", 30));
     
  // Add buttons to the GUI for controlling LED matrix display
  cp5.addButton("displayHeart")
    .setPosition(buttonX, buttonY + 1 * buttonSpacing)
    .setSize(buttonWidth, buttonHeight)
    .setLabel("Display Heart")
    .getCaptionLabel().setFont(createFont("Arial", 35));

  cp5.addButton("displaySmiley")
    .setPosition(buttonX, buttonY + 2 * buttonSpacing)
    .setSize(buttonWidth, buttonHeight)
    .setLabel("Display Smiley")
    .getCaptionLabel().setFont(createFont("Arial", 35));

  cp5.addButton("displayW5")
    .setPosition(buttonX, buttonY + 3 *  buttonSpacing)
    .setSize(buttonWidth, buttonHeight)
    .setLabel("Display W5")
    .getCaptionLabel().setFont(createFont("Arial", 40));
        
}


// Draw function runs continuously
void draw()
{
  background(0);
  fill(0);
  
  // Read data from the Arduino

  data = myClient.readStringUntil('\n');
  
  // Check if data is received
  if (data != null)
  {
    // Process different types of data received from Arduino
    
      println(data);
      
      //cp5.get(Textlabel.class, "distanceLabel").setText("Distance travelled: " + data.substring(1));
      // cp5.get(Textlabel.class, "objectLabel").setText("Object Distance: " + data.substring(1));
      
  };
    
    /*
    else if (data[0] == )
    {
      println(data);
    }
    else if (data[0] == )
    {
      println(data);
    }
    */ 
        
  };



// Function to send commands to the Arduino
void sendCommand(char command)
{

  // Load the URL to send the command to the Arduino
    if (myClient != null) 
    {
      myClient.write(command);
      // println("Sent Command");
      // println("Sending command: " + command);
      // println(myClient.ip());
    }
    else
    {
      println("Shut up");
    }
}


void StopStart(boolean theFlag)
{
  if(theFlag) 
  {
    sendCommand(startURL);
    cp5.setColorBackground(color(0,255,0));
  } 
  else 
  {
    sendCommand(stopURL);
    cp5.setColorBackground(color(255,0,0));
  }
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
