#include <Arduino.h>
#include <FlexCAN_T4.h>

/*
Upload this code to test if the board can receive and transmit messages from CAN bus.
*/


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> myCan1;

static CAN_message_t msg;   // 

void send_data()
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

void read_data()
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

void setup()
{
  delay(1000);
  Serial.begin(115200);
  myCan1.begin();
  myCan1.setBaudRate(250000);

}

void loop()
{
  read_data();
  send_data();
  delay(1);

}