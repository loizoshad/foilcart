#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

/*
Upload this code to test if the board can receive and transmit messages from CAN bus and at the same time read the GNSS measurements.
*/


SFE_UBLOX_GNSS myGNSS;  // GNSS object
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> myCan1;
static CAN_message_t msg;


void write_can()
{
  // Serial.println("Sending Data!");
  // Send a message on CAN bus
  msg.id = 0x03;
  msg.len = 8;
  msg.buf[0] = 0x01;
  msg.buf[1] = 0x02;
  msg.buf[2] = 0x03;
  msg.buf[3] = 0x04;
  msg.buf[4] = 0x05;
  msg.buf[5] = 0x06;
  msg.buf[6] = 0x07;
  msg.buf[7] = 0x08;
  myCan1.write(msg);

}

void read_can()
{
  // Serial.println("Reading Data!");
  // Check if a message is received
  while (myCan1.read(msg) != 0)
  {
    Serial.print("ID: ");
    Serial.println(msg.id, HEX);

    Serial.print("Data: ");
    for (int i = 0; i < msg.len; i++)
    {
      Serial.print(msg.buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void init_gnss()
{
  Wire.begin();

  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
}


void setup()
{
  delay(1000);

  // Serial monitor for debugging purposes
  Serial.begin(115200);

  // Setup CAN bus communication
  myCan1.begin();
  myCan1.setBaudRate(250000);

  // Setup GNSS
  init_gnss();
}

void loop()
{
  read_can();
  write_can();

  long ground_speed = myGNSS.getGroundSpeed(); // Ground speed in mm/s
  float groundSpeed = static_cast<float>(ground_speed) / 1000.0; // Convert to m/s
  Serial.print(F(" Ground Speed: "));
  Serial.print(groundSpeed, 3);
  Serial.println(F(" m/s"));

  delay(100);

}