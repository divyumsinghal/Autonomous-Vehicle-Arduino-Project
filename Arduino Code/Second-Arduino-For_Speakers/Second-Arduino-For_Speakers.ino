#include <SD.h>     // need to include the SD library
#include <TMRpcm.h> //  Lib to play wav file
#include <SPI.h>    // need to include the SPI library

// Inculde the library for Talkie in order to set up the speaker
#include "Talkie.h"

// Simple words
#include "Vocab_US_Large.h"
#include "Vocab_US_Clock.h"
#include "Vocab_Special.h"
#include "Vocab_US_Acorn.h"
#include "Vocab_US_TI99.h"

Talkie voice; // Create a talkie object

TMRpcm Speaker; // create an object for use in this sketch

const int SD_ChipSelectPin = 6; // using digital pin 6 for the SD card select pin
const int SignalInput = A7;     // using analog pin 0 for the audio signal input

/* Say any number between -999,999 and 999,999 */
void sayNumber(long n) {
    if (n < 0) {
        voice.say(sp2_MINUS);
        sayNumber(-n);
    } else if (n == 0) {
        voice.say(sp2_ZERO);
    } else {
        if (n >= 1000) {
            int thousands = n / 1000;
            sayNumber(thousands);
            voice.say(sp2_THOUSAND);
            n %= 1000;
            if ((n > 0) && (n < 100))
                voice.say(sp2_AND);
        }
        if (n >= 100) {
            int hundreds = n / 100;
            sayNumber(hundreds);
            voice.say(sp2_HUNDRED);
            n %= 100;
            if (n > 0)
                voice.say(sp2_AND);
        }
        if (n > 19) {
            int tens = n / 10;
            switch (tens) {
            case 2:
                voice.say(sp2_TWENTY);
                break;
            case 3:
                voice.say(sp2_THIR_);
                voice.say(sp2_T);
                break;
            case 4:
                voice.say(sp2_FOUR);
                voice.say(sp2_T);
                break;
            case 5:
                voice.say(sp2_FIF_);
                voice.say(sp2_T);
                break;
            case 6:
                voice.say(sp2_SIX);
                voice.say(sp2_T);
                break;
            case 7:
                voice.say(sp2_SEVEN);
                voice.say(sp2_T);
                break;
            case 8:
                voice.say(sp2_EIGHT);
                voice.say(sp2_T);
                break;
            case 9:
                voice.say(sp2_NINE);
                voice.say(sp2_T);
                break;
            }
            n %= 10;
        }
        switch (n) {
        case 1:
            voice.say(sp2_ONE);
            break;
        case 2:
            voice.say(sp2_TWO);
            break;
        case 3:
            voice.say(sp2_THREE);
            break;
        case 4:
            voice.say(sp2_FOUR);
            break;
        case 5:
            voice.say(sp2_FIVE);
            break;
        case 6:
            voice.say(sp2_SIX);
            break;
        case 7:
            voice.say(sp2_SEVEN);
            break;
        case 8:
            voice.say(sp2_EIGHT);
            break;
        case 9:
            voice.say(sp2_NINE);
            break;
        case 10:
            voice.say(sp2_TEN);
            break;
        case 11:
            voice.say(sp2_ELEVEN);
            break;
        case 12:
            voice.say(sp2_TWELVE);
            break;
        case 13:
            voice.say(sp2_THIR_);
            voice.say(sp2__TEEN);
            break;
        case 14:
            voice.say(sp2_FOUR);
            voice.say(sp2__TEEN);
            break;
        case 15:
            voice.say(sp2_FIF_);
            voice.say(sp2__TEEN);
            break;
        case 16:
            voice.say(sp2_SIX);
            voice.say(sp2__TEEN);
            break;
        case 17:
            voice.say(sp2_SEVEN);
            voice.say(sp2__TEEN);
            break;
        case 18:
            voice.say(sp2_EIGHT);
            voice.say(sp2__TEEN);
            break;
        case 19:
            voice.say(sp2_NINE);
            voice.say(sp2__TEEN);
            break;
        }
    }
}

/**
 * @brief The quantisation level for the audio signal input.
 *
 * This variable determines the level of quantisation applied to the signal input
 *
 * @note The valid range for this variable is from 1 to 254.
 */
const int quantisation_level = 20;

/**
 *
 * Initializes the Arduino board and sets up the necessary configurations.
 * This function is called only once when the Arduino powers up or is reset.
 *
 */
void setup()
{
  Speaker.speakerPin = 9; // pin 9 for Aduino Pro Mini , any pin with PWM fonction will work too

  Serial.begin(9600);

  pinMode(SignalInput, INPUT); // set the audio signal from Arduino R4 input as an input

  while (!SD.begin(SD_ChipSelectPin))
  {
    // see if the card is present and can be initialized:
    Serial.println("SD fail");
    return; // don't do anything more if not
  }

  // SD card is connected and initialized
  Serial.print("SD connected");
  Speaker.play("music.wav"); // the sound file "music" will play each time the arduino powers up, or is reset
  Speaker.volume(7);

#if defined(TEENSYDUINO)
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH); // Enable Amplified PROP shield
#endif

  // voice.say(sp2_MILLI); //voice will say stuff

  voice.say(spc_GOOD);
  voice.say(spc_MORNING);
  voice.say(spPAUSE1);
  voice.say(spc_THE);
  voice.say(spc_TIME);
  voice.say(spc_IS);
  voice.say(spc_ELEVEN);
  voice.say(spc_THIRTY);
  voice.say(spc_SIX);
  voice.say(spc_A_M_);
  voice.say(spPAUSE1);

  voice.say(spa_THIS);
  voice.say(spa_IS);
  voice.say(spa_THE);
  voice.say(spa_ACORN);
  voice.say(spa_COMPUTER);
  voice.say(spa__Z);
  voice.say(spa_FILE);
  voice.say(spa_FROM);
  voice.say(spa_NINE_);
  voice.say(spa__TEEN);
  voice.say(spa_EIGH_);
  voice.say(spa_T);
  voice.say(spa_THREE);
  voice.say(spPAUSE1);
}

/**
 * The main loop of the Arduino program.
 * Reads the audio signal input and plays corresponding sound files based on the input level.
 */
void loop()
{
  int input_10 = analogRead(SignalInput); // read the audio signal input

  double input_8 = map(input_10,
                       0,
                       1023,
                       0,
                       255); // scale the input to 8 bits

  /*
   * The audio signal input is quantised into 12 levels.
   * The input is compared with the quantisation levels to determine which sound file to play.
   * The correspponding sound file will play if the input is within the range of the quantisation level.
   * If the input is less than the first quantisation level, the sound file will stop playing.
   * The sound files should be stored in the root directory of the SD card.
   * The sound files should be in the .wav format.
   *
   */

  if (input_8 <= quantisation_level)
  {
    Speaker.stopPlayback(); // stop playing the sound file
  }
  else if (input_8 > quantisation_level && input_8 <= 2 * quantisation_level)
  {
    Speaker.play("music1.wav"); // the sound file "music" will play
  }
  else if (input_8 > 2 * quantisation_level && input_8 <= 3 * quantisation_level)
  {
    Speaker.play("music2.wav"); // the sound file "music" will play
  }
  else if (input_8 > 3 * quantisation_level && input_8 <= 4 * quantisation_level)
  {
    Speaker.play("music3.wav"); // the sound file "music" will play
  }
  else if (input_8 > 4 * quantisation_level && input_8 <= 5 * quantisation_level)
  {
    Speaker.play("music4.wav"); // the sound file "music" will play
  }
  else if (input_8 > 5 * quantisation_level && input_8 <= 6 * quantisation_level)
  {
    Speaker.play("music5.wav"); // the sound file "music" will play
  }
  else if (input_8 > 6 * quantisation_level && input_8 <= 7 * quantisation_level)
  {
    Speaker.play("music6.wav"); // the sound file "music" will play
  }
  else if (input_8 > 7 * quantisation_level && input_8 <= 8 * quantisation_level)
  {
    Speaker.play("music7.wav"); // the sound file "music" will play
  }
  else if (input_8 > 8 * quantisation_level && input_8 <= 9 * quantisation_level)
  {
    Speaker.play("music8.wav"); // the sound file "music" will play
  }
  else if (input_8 > 9 * quantisation_level && input_8 <= 10 * quantisation_level)
  {
    Speaker.play("music9.wav"); // the sound file "music" will play
  }
  else if (input_8 > 10 * quantisation_level && input_8 <= 11 * quantisation_level)
  {
    Speaker.play("music10.wav"); // the sound file "music10" will play
  }
  else if (input_8 > 11 * quantisation_level && input_8 <= 12 * quantisation_level)
  {
    Speaker.play("music11.wav"); // the sound file "music" will play
  }
  else if (input_8 > 12 * quantisation_level)
  {
    Speaker.play("music12.wav"); // the sound file "music" will play
  }
}
