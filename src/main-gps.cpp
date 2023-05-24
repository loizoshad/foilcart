#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

void setup()
{
  Serial.begin(115200);
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages

}

void loop()
{
  // myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.
  
  long latitude = myGNSS.getLatitude();                               // Lattitude in degrees decimal * 10^-7
  float latitudeDegrees = static_cast<float>(latitude) / 10000000.0;  // Convert to degrees
  Serial.print(F("Latitude: "));
  Serial.print(latitudeDegrees, 7);

  long longitude = myGNSS.getLongitude();                               // Longitude in degrees decimal * 10^-7
  float longitudeDegrees = static_cast<float>(longitude) / 10000000.0;  // Convert to degrees
  Serial.print(F(" Longitude: "));
  Serial.print(longitudeDegrees, 7);

  long ground_speed = myGNSS.getGroundSpeed(); // Ground speed in mm/s
  float groundSpeed = static_cast<float>(ground_speed) / 1000.0; // Convert to m/s
  Serial.print(F(" Ground Speed: "));
  Serial.print(groundSpeed, 3);
  Serial.println(F(" m/s"));

  delay(100); //Don't pound too hard on the I2C bus
}
