#include <Wire.h>

void setup() {
  Wire.begin(2, 15); // SDA = GPIO2, SCL = GPIO15 
  Serial.begin(115200);
  delay(2000); // Đợi Serial ổn định
  Serial.println("\nI2C Scanner starting...");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning for I2C devices...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    }
  }

  if (nDevices == 0) Serial.println("No I2C devices found\n");
  else Serial.println("Scan done\n");

  delay(3000); // Scan mỗi 3 giây
}
