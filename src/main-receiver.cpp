#include <Arduino.h>
#include <FlexCAN_T4.h>

/*
Upload this code to test if the board can receive messages from CAN bus
*/


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> myCan1;

static CAN_message_t msg03;       // Received message 
static CAN_message_t msg50;   // Acknowledgement message


void setup()
{
  Serial.begin(115200);
  delay(1000);
  myCan1.begin();
  myCan1.setBaudRate(250000);

}

void sendAcknowledgement(int srcAdr, int insAdr, long insVal)
{
  
  // Acknowledgement
  msg50.flags.extended = 0;
  msg50.id = 0x50;
  msg50.len = 8;
  msg50.buf[0] = highByte(srcAdr);
  msg50.buf[1] = lowByte(srcAdr);
  msg50.buf[2] = highByte(insAdr);
  msg50.buf[3] = lowByte(insAdr);
  msg50.buf[4] = insVal >> 24;
  msg50.buf[5] = insVal >> 16;
  msg50.buf[6] = insVal >> 8;
  msg50.buf[7] = insVal;
  myCan1.write(msg50);
}

void loop()
{

  // Check if a message is received
  while (myCan1.read(msg03) != 0)
  {
    Serial.print("ID: ");
    Serial.println(msg03.id, HEX);

    Serial.print("Data: ");
    for (int i = 0; i < msg03.len; i++)
    {
      Serial.print(msg03.buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // Send an acknowledgement message
    sendAcknowledgement(0x03, 0x01, 0x01); // 0x03 is the source address, 0x01 is the instruction address, 0x01 is the instruction value


  }

}

