// #include <Arduino.h>
// #include <FlexCAN_T4.h>

// // Transmitter


// FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> myCan1;

// static CAN_message_t msg;   // 


// void setup()
// {
//   delay(1000);
//   myCan1.begin();
//   myCan1.setBaudRate(1000000);

// }

// void loop()
// {

//   // Send a message on CAN bus
//   msg.id = 0x03;
//   msg.len = 8;
//   msg.buf[0] = 0x01;
//   msg.buf[1] = 0x02;
//   msg.buf[2] = 0x03;
//   msg.buf[3] = 0x04;
//   msg.buf[4] = 0x05;
//   msg.buf[5] = 0x06;
//   msg.buf[6] = 0x07;
//   msg.buf[7] = 0x08;
//   myCan1.write(msg);
//   delay(1000);

// }