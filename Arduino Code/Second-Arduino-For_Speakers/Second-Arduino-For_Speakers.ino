#include <SD.h>     // need to include the SD library
#include <TMRpcm.h> //  Lib to play wav file
#include <SPI.h>

TMRpcm tmrpcm; // create an object for use in this sketch

const int SD_ChipSelectPin = 6; // using digital pin 6 for the SD card select pin
const int SignalInput = A0;     // using analog pin 0 for the audio signal input

const int quantisation_level = 20; // set the quantisation level for the audio signal input

void setup()
{
  tmrpcm.speakerPin = 9; // pin 9 for Aduino Pro Mini , any pin with PWM fonction will work too

  Serial.begin(9600);

  pinMode(SignalInput, INPUT); // set the audio signal input as an input

  if (!SD.begin(SD_ChipSelectPin))
  { // see if the card is present and can be initialized:
    Serial.println("SD fail");
    return; // don't do anything more if not
  }
  else
  {
    Serial.print("SD connected");
    tmrpcm.play("music.wav"); // the sound file "music" will play each time the arduino powers up, or is reset
    tmrpcm.volume(7);
  }
}

void loop()
{
  int input = analogRead(SignalInput); // read the audio signal input

  if (input <= quantisation_level)
  {
    tmrpcm.stopPlayback(); // stop playing the sound file
  }
  else if (input > quantisation_level && input <= 2 * quantisation_level)
  {
    tmrpcm.play("music1.wav"); // the sound file "music" will play
  }
  else if (input > 2 * quantisation_level && input <= 3 * quantisation_level)
  {
    tmrpcm.play("music2.wav"); // the sound file "music" will play
  }
  else if (input > 3 * quantisation_level && input <= 4 * quantisation_level)
  {
    tmrpcm.play("music3.wav"); // the sound file "music" will play
  }
  else if (input > 4 * quantisation_level && input <= 5 * quantisation_level)
  {
    tmrpcm.play("music4.wav"); // the sound file "music" will play
  }
  else if (input > 5 * quantisation_level && input <= 6 * quantisation_level)
  {
    tmrpcm.play("music5.wav"); // the sound file "music" will play
  }
  else if (input > 6 * quantisation_level && input <= 7 * quantisation_level)
  {
    tmrpcm.play("music6.wav"); // the sound file "music" will play
  }
  else if (input > 7 * quantisation_level && input <= 8 * quantisation_level)
  {
    tmrpcm.play("music7.wav"); // the sound file "music" will play
  }
  else if (input > 8 * quantisation_level && input <= 9 * quantisation_level)
  {
    tmrpcm.play("music8.wav"); // the sound file "music" will play
  }
  else if (input > 9 * quantisation_level && input <= 10 * quantisation_level)
  {
    tmrpcm.play("music9.wav"); // the sound file "music" will play
  }
  else if (input > 10 * quantisation_level && input <= 11 * quantisation_level)
  {
    tmrpcm.play("music10.wav"); // the sound file "music10" will play
  }
  else if (input > 11 * quantisation_level && input <= 12 * quantisation_level)
  {
    tmrpcm.play("music11.wav"); // the sound file "music" will play
  }
  else if (input > 12 * quantisation_level)
  {
    tmrpcm.play("music12.wav"); // the sound file "music" will play
  }
}
