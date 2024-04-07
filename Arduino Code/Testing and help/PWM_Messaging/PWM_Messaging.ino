
// MusicFile
const int SpeakerPin = 6;

const int base = 10;
const int quantisation_level = 20;

// Define an enumeration for sounds
/**
 *
 * Stores the files to be played on the speaker.
 * these are sent to the other arduino to play the sound
 *
 */
enum MusicFile {
  NoSound = 0,
  IDAccepted = base + quantisation_level,
  ObjectDetected = base + 2 * quantisation_level,
  GUIstop = base + 3 * quantisation_level,
  SignSTop = base + 4 * quantisation_level,
  Right = base + 5 * quantisation_level,
  Left = base + 6 * quantisation_level,
  Sound7 = base + 7 * quantisation_level,
  Sound8 = base + 8 * quantisation_level,
  Sound9 = base + 9 * quantisation_level,
  Sound10 = base + 10 * quantisation_level,
  Sound11 = base + 11 * quantisation_level,
  Sound12 = 255

};

// Play MusicFile on Speaker
/**
 * Plays a sound on the speaker.
 *
 * Sends a value to the  other arduino of which file to play
 * 0 -> Sound1
 * 1 -> Sound2
 * 2 -> Sound3
 * 3 -> Sound4
 *
 * @param command The sound command to be played.
 */
void PlaySoundOnSpeaker(MusicFile command) {
  Serial.println(int(command));

  // Play the sound on the speaker
  analogWrite(SpeakerPin, int(command));

  Serial.println(int(command));
  Serial.println(int(command));


  // delay(100); // Delay to allow the sound to be detetected
}



void initialiseStuff() {

  pinMode(SpeakerPin, OUTPUT);
}


void setup() {

  Serial.begin(9600);

  Serial.println(".");
  Serial.println(".");
  Serial.println(".");
  Serial.println(".");
  Serial.println(".");
}


void loop() {

  /*

  char input = Serial.read();

  switch (input) {

    case 'a':

      PlaySoundOnSpeaker(IDAccepted);

      break;

    case 'b':

      PlaySoundOnSpeaker(ObjectDetected);

      break;

    case 'c':

      PlaySoundOnSpeaker(GUIstop);

      break;

    case 'd':

      PlaySoundOnSpeaker(SignSTop);

      break;

    case 'e':

      PlaySoundOnSpeaker(Right);

      break;

    case 'f':

      PlaySoundOnSpeaker(Left);

      break;

    default:

      Serial.print("-");

      break;
  }
  */

  delay(500);
  

  analogWrite(6, 5);

  Serial.print(".");

}

// End of Code