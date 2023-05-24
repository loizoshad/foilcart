// #include <FlexCAN_T4.h>
// #include <elapsedMillis.h>
// #include <stdint.h>

// // CAN Message schema, see Dropbox

// FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// static CAN_message_t msg;
// // CAN statics

// static CAN_message_t msg50; //Acknowledgement

// static CAN_message_t msg120; //Senix altitude fore 
// static CAN_message_t msg121; //Senix altitude aft

// static CAN_message_t msg200; // EKF states 1-2
// static CAN_message_t msg201; // EKF states 3-4
// static CAN_message_t msg202; // EKF states 5-6
// static CAN_message_t msg203; // EKF states 7-8
// static CAN_message_t msg204; // EKF states 9-10
// static CAN_message_t msg205; // EKF states 11-12

// void init_can() 
// {

//     Can1.begin();
//     Can1.setBaudRate(250000);

//     //--------------------------------------
//     // Acknowledgement
//     msg50.flags.extended = 0;
//     msg50.id = 0x50;
//     msg50.len = 8;
//     msg50.buf[0] = 100;
//     msg50.buf[1] = 0;
//     msg50.buf[2] = 0;
//     msg50.buf[3] = 0;
//     msg50.buf[4] = 0;
//     msg50.buf[5] = 0;
//     msg50.buf[6] = 0;
//     msg50.buf[7] = 0;

// }


// void read_can( float* fore_alt, float* aft_alt, float* wing_angle, float* rudder_angle, float* elevator_angle, float* throttle)
// {                  
//     CAN_message_t inMsg;
//     while ( Can1.read(inMsg)!=0 ) {

//         // Serial.print("CHECKING IF THERE ARE NEW MESSAGES IN CAN BUS)\n");
        
//         switch(inMsg.id) 
//         {
//         // Senix sensors data:
//         case 0x120: // Fore Senix sensor altitude
//         {
//             int16_t fore_alt_int = (int16_t) (inMsg.buf[0] << 8 | inMsg.buf[1]);
//             *fore_alt = (float) fore_alt_int;   // TODO: Implement the correct conversion
//             // for (int i = 0; i < inMsg.len; i++)
//             // {
//             //     Serial.print(inMsg.buf[i], HEX);
//             //     Serial.print(" ");
//             // }      
//             // Serial.print("fore_alt: ");
//             // Serial.println(*fore_alt);                  
//             // break;
//         }
//         case 0x121: // Aft Senix sensor altitude
//         {
//             int16_t aft_alt_int = (int16_t) (inMsg.buf[0] << 8 | inMsg.buf[1]);
//             *aft_alt = (float) aft_alt_int;   // TODO: Implement the correct conversion
//             // for (int i = 0; i < inMsg.len; i++)
//             // {
//             //     Serial.print(inMsg.buf[i], HEX);
//             //     Serial.print(" ");
//             // }
//             // Serial.print("aft_alt: ");
//             // Serial.println(*aft_alt);
//             break;
//         }
//         // Control input data:
//         case 0x105: // Control input (Roll, Elevator, Throttle, Rudder)
//         {
//             break;
//         }
//         case 0x106: // Control input (Wing angle, Flaps, Control type, Ch8)
//         {
//             break;
//         }
//         default:
//         {
//             Serial.print("ERROR: inMsd.id = ");
//             Serial.println(inMsg.id, HEX);
//             break;            
//         }
//         }
//     }
// }


