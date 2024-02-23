// Import the ControlP5 and processing.net libraries
import controlP5.*;

// Declare ControlP5 object
ControlP5 cp5;

import processing.net.*;

// Declare a Client object for network communication
Client myClient;

// Arduino's IP address (replace with actual IP address)
String serverAddress = "192.168.0.114";

// String to store received data from the Arduino
String data;

// URLs for different car commands
char startURL = '1';
char stopURL = '0';
char displayHeartURL = 'H';
char displaySmileyURL = 'S';
char displayW5URL = 'W';
String distanceTravelledURL = "A"; 
String nearestObjectURL = "B";

// Setup function called once at the beginning
void setup()
{

  // Initialize client for communication with Arduino
  myClient = new Client(this, serverAddress, 5200);

  // Set the size of the Processing window
  size(300, 1000);

  // Initialize controlP5
  cp5 = new ControlP5(this);


  // Positioning variables for buttons
  int buttonX = 50;
  int buttonY = 50;
  int buttonWidth = 200;
  int buttonHeight = 100;
  int buttonSpacing = 125;
  
  // Add a "Start" button to the GUI
  cp5.addButton("start")
    .setPosition(buttonX, buttonY)
    .setSize(buttonWidth, buttonHeight)
    .setLabel("Start");
    
   cp5.addButton("stop")
    .setPosition(buttonX, buttonY + buttonSpacing)
    .setSize(buttonWidth, buttonHeight)
    .setLabel("Stop");
    
    
  // Add buttons to the GUI for controlling LED matrix display
  cp5.addButton("displayHeart")
    .setPosition(buttonX, buttonY + 2 * buttonSpacing)
    .setSize(buttonWidth, buttonHeight)
    .setLabel("Display Heart");

  cp5.addButton("displaySmiley")
    .setPosition(buttonX, buttonY + 3 * buttonSpacing)
    .setSize(buttonWidth, buttonHeight)
    .setLabel("Display Smiley");

  cp5.addButton("displayW5")
    .setPosition(buttonX, buttonY + 4 *  buttonSpacing)
    .setSize(buttonWidth, buttonHeight)
    .setLabel("Display W5");
    
   cp5.addTextlabel("distanceLabel")
     .setPosition(buttonX, buttonY + 5 *  buttonSpacing)
     .setColor(color(255))
     .setFont(createFont("Arial", 12));
    
   cp5.addTextlabel("objectLabel")
     .setPosition(buttonX, buttonY + 6 *  buttonSpacing)
     .setColor(color(255))
     .setFont(createFont("Arial", 12));
}


// Draw function runs continuously
void draw()
{
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
      println("Sending command: " + command);
      println(myClient.ip());
    }
    else
    {
      println("Shut up");
    }
}

// Start the car

public void start()
{
  sendCommand(startURL);
}

// Stop the car

public void stop()
{
  sendCommand(stopURL);
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
