#include <Wire.h>

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;

  Serial.println("Scanning I2C devices...");
}

void loop()
{
  scanI2C();
}

void scanI2C()
{
  byte error, address;
  int devices = 0;

  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      devices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (devices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("Scanning complete\n");
}
