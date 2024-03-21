# 1 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Testing and help\\Check_I2C_Connection\\Check_I2C_Connection.ino"
# 2 "C:\\Users\\divyu\\OneDrive - Trinity College Dublin\\Desktop\\Buggy\\Autonomous-Vehicle-Arduino-Project\\Arduino Code\\Testing and help\\Check_I2C_Connection\\Check_I2C_Connection.ino" 2

void setup() {
  _UART1_.begin(9600);
  while (!_UART1_);

  _UART1_.println("Scanning I2C devices...");
  scanI2C();
}

void loop() {
}

void scanI2C() {
  byte error, address;
  int devices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      _UART1_.print("I2C device found at address 0x");
      if (address < 16)
        _UART1_.print("0");
      _UART1_.print(address, 16);
      _UART1_.println(" !");
      devices++;
    } else if (error == 4) {
      _UART1_.print("Unknown error at address 0x");
      if (address < 16)
        _UART1_.print("0");
      _UART1_.println(address, 16);
    }
  }

  if (devices == 0)
    _UART1_.println("No I2C devices found\n");
  else
    _UART1_.println("Scanning complete\n");
}
