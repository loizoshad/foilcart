// /*----------------------------------------------------------------------------
//  *     Filename       : can_com.cpp
//  *     Purpose        : Program to read from CAN-bus
//  *
//  *     Programmer    : Adam Thålin, 2022
//  *     Modified by   : Loizos Hadjiloizou, 2022
//  *     Date          : 20220802
//  *     Version       : 0
//  *
//  *--------------------------------------------------------------------------*/

// #include <FlexCAN_T4.h>
// #include <elapsedMillis.h>
// #include <stdint.h>

// #define PI_ 3.1416f     // pi (3.14159265358979323846...)
// #define G_ 9.82f        // Gravitational acceleration in Stockholm

// // CAN Message schema, see Dropbox


// FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// static CAN_message_t msg;
// // CAN statics

// static CAN_message_t msg50; //Acknowledgement

// static CAN_message_t msg101; //Remote control ch 1-4
// static CAN_message_t msg102; //Remote control ch 5-8

// static CAN_message_t msg105; //Control output ch 1-4
// static CAN_message_t msg106; //Control output ch 5-8


// static CAN_message_t msg120; //Senix altitude fore 
// static CAN_message_t msg121; //Senix altitude aft

// static CAN_message_t msg130; //IMU Eulers X&Y
// static CAN_message_t msg131; //IMU Euler Z
// static CAN_message_t msg132; //IMU Rot X&Y
// static CAN_message_t msg133; //IMU Rot Z

// static CAN_message_t msg134; // Acc_XY
// static CAN_message_t msg135; // Acc_Z
// static CAN_message_t msg136; // Ang vel X&Y
// static CAN_message_t msg137; // Ang vel Z
// static CAN_message_t msg138; // Inc X&Y
// static CAN_message_t msg139; // Inc Z

// static CAN_message_t msg200; // EKF states 1-2
// static CAN_message_t msg201; // EKF states 3-4
// static CAN_message_t msg202; // EKF states 5-6
// static CAN_message_t msg203; // EKF states 7-8
// static CAN_message_t msg204; // EKF states 9-10
// static CAN_message_t msg205; // EKF states 11-12

// void canInit() 
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

//     //--------------------------------------
//     // RC inputs 1-4
//     msg101.flags.extended = 0;
//     msg101.id = 0x101;
//     msg101.len = 8;
//     msg101.buf[0] = (int16_t) 1100;
//     msg101.buf[1] = (int16_t) 1100;
//     msg101.buf[2] = (int16_t) 1100;
//     msg101.buf[3] = (int16_t) 1100;
//     msg101.buf[4] = (int16_t) 1100;
//     msg101.buf[5] = (int16_t) 1100;
//     msg101.buf[6] = (int16_t) 1100;
//     msg101.buf[7] = (int16_t) 1100;

//     //--------------------------------------
//     // RC inputs 5-8
//     msg102.flags.extended = 0;
//     msg102.id = 0x102;
//     msg102.len = 8;
//     msg102.buf[0] = 1100;
//     msg102.buf[1] = 1100;
//     msg102.buf[2] = 1100;
//     msg102.buf[3] = 1100;
//     msg102.buf[4] = 1100;
//     msg102.buf[5] = 1100;
//     msg102.buf[6] = 1100;
//     msg102.buf[7] = 1100;

//     //--------------------------------------
//     // Control outputs 1-4
//     msg105.flags.extended = 0;
//     msg105.id = 0x105;
//     msg105.len = 8;
//     msg105.buf[0] = 1100;
//     msg105.buf[1] = 1100;
//     msg105.buf[2] = 1100;
//     msg105.buf[3] = 1100;
//     msg105.buf[4] = 1100;
//     msg105.buf[5] = 1100;
//     msg105.buf[6] = 1100;
//     msg105.buf[7] = 1100;

//     //--------------------------------------
//     // Control outputs 5-8
//     msg106.flags.extended = 0;
//     msg106.id = 0x106;
//     msg106.len = 8;
//     msg106.buf[0] = 1100;
//     msg106.buf[1] = 1100;
//     msg106.buf[2] = 1100;
//     msg106.buf[3] = 1100;
//     msg106.buf[4] = 1100;
//     msg106.buf[5] = 1100;
//     msg106.buf[6] = 1100;
//     msg106.buf[7] = 1100;

//     //--------------------------------------  
//     //   // EKF CAN-messages:

//     //   // EKF 1-2
//     //   msg200.flags.extended = 0;
//     //   msg200.id = 0x200;
//     //   msg200.len = 8;
//     //   msg200.buf[0] = 0;
//     //   msg200.buf[1] = 0;
//     //   msg200.buf[2] = 0;
//     //   msg200.buf[3] = 0;
//     //   msg200.buf[4] = 0;
//     //   msg200.buf[5] = 0;
//     //   msg200.buf[6] = 0;
//     //   msg200.buf[7] = 0;

//     //   // EKF 3-4
//     //   msg201.flags.extended = 0;
//     //   msg201.id = 0x201;
//     //   msg201.len = 8;
//     //   msg201.buf[0] = 0;
//     //   msg201.buf[1] = 0;
//     //   msg201.buf[2] = 0;
//     //   msg201.buf[3] = 0;
//     //   msg201.buf[4] = 0;
//     //   msg201.buf[5] = 0;
//     //   msg201.buf[6] = 0;
//     //   msg201.buf[7] = 0;

//     //   //EKF 5-6
//     //   msg202.flags.extended = 0;
//     //   msg202.id = 0x202;
//     //   msg202.len = 8;
//     //   msg202.buf[0] = 0;
//     //   msg202.buf[1] = 0;
//     //   msg202.buf[2] = 0;
//     //   msg202.buf[3] = 0;
//     //   msg202.buf[4] = 0;
//     //   msg202.buf[5] = 0;
//     //   msg202.buf[6] = 0;
//     //   msg202.buf[7] = 0;

//     //   //EKF 7-8
//     //   msg203.flags.extended = 0;
//     //   msg203.id = 0x203;
//     //   msg203.len = 8;
//     //   msg203.buf[0] = 0;
//     //   msg203.buf[1] = 0;
//     //   msg203.buf[2] = 0;
//     //   msg203.buf[3] = 0;
//     //   msg203.buf[4] = 0;
//     //   msg203.buf[5] = 0;
//     //   msg203.buf[6] = 0;
//     //   msg203.buf[7] = 0;

//     //   //EKF 9-10
//     //   msg204.flags.extended = 0;
//     //   msg204.id = 0x204;
//     //   msg204.len = 8;
//     //   msg204.buf[0] = 0;
//     //   msg204.buf[1] = 0;
//     //   msg204.buf[2] = 0;
//     //   msg204.buf[3] = 0;
//     //   msg204.buf[4] = 0;
//     //   msg204.buf[5] = 0;
//     //   msg204.buf[6] = 0;
//     //   msg204.buf[7] = 0;

//     //   //EKF 11-12
//     //   msg205.flags.extended = 0;
//     //   msg205.id = 0x205;
//     //   msg205.len = 8;
//     //   msg205.buf[0] = 0;
//     //   msg205.buf[1] = 0;
//     //   msg205.buf[2] = 0;
//     //   msg205.buf[3] = 0;
//     //   msg205.buf[4] = 0;
//     //   msg205.buf[5] = 0;
//     //   msg205.buf[6] = 0;
//     //   msg205.buf[7] = 0;


// }  //CANinit

// // -------------------------------------------------------------

// ///////////////////////////////////////////////////////////////////////////
// //      FUNCTION NAME : read_data
// //        DESCRIPTION : Converting byte messages to values.
// //                      Teensy does not seem to handle three byte integers.
// //                      Memory is not an issue right now, but would be waste
// //                      to use four byte ints...
// //                 IN : float*
// //                OUT : 
// ///////////////////////////////////////////////////////////////////////////

// void read_data( float* vx, float* ax, float* ay, float* az,
//                 float* gx, float* gy, float* gz ) 
// {
                   
//     CAN_message_t inMsg;
//     while ( Can1.read(inMsg)!=0 ) {

//         // Serial.print("CHECKING IF THERE ARE NEW MESSAGES IN CAN BUS)\n");
        
//         switch(inMsg.id) 
//         {
//         // GPS Data:
//         case 0x99: // GPS Speed over Ground
//             *vx = ((float)((int32_t)((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8)) | (inMsg.buf[3])))/1000000.0f;
//             break;

//         default:
//             Serial.print("ERROR: inMsd.id = ");
//             Serial.println(inMsg.id, HEX);
//             break;
//         }
//     }
// }

// void read_inputs( float vx, float z[24] ) 
// {

//     CAN_message_t inMsg;

//     int16_t rollrudder = 0;
//     int16_t elevator = 0;
//     int16_t throttle = 0;
//     // int16_t rud = 0;
//     int16_t mainWing = 0;
//     // int16_t flaps = 0;
//     // int16_t ctrlType = 0;
//     // int16_t movingdir = 0;

//     while ((Can1.read(inMsg)!=0)) 
//     {  
//         switch(inMsg.id) 
//         {
//         case 0x105:
//             rollrudder = (int16_t) ( inMsg.buf[0]<<8 | inMsg.buf[1] );
//             elevator = (int16_t) ( inMsg.buf[2]<<8 | inMsg.buf[3] );
//             throttle = (int16_t) ( inMsg.buf[4]<<8 | inMsg.buf[5] );
//             // rud = (int16_t) ( inMsg.buf[6]<<8 | inMsg.buf[7] );
//             break;
//         case 0x106:
//             mainWing = (int16_t) ( inMsg.buf[0]<<8 | inMsg.buf[1] );
//             // flaps    = (int16_t) ( inMsg.buf[0]<<8 | inMsg.buf[1] );
//             // ctrlType = (int16_t) ( inMsg.buf[4]<<8 | inMsg.buf[5] );
//             // movingdir = (int16_t) (inMsg.buf[6]<<8 | inMsg.buf[7] );
//             break;
//         }
//     }

//     // Mathematical gymnastics:
//     // kx + m, linear function
//     float k = 1.0f;
//     float m = -0.0f;

//     float Awing = 0.01f;
//     float Aelevator = 0.005f;
//     float Arudder = 0.005f;

//     // Motors (No idea how to unpack this)
//     z[16] = (float) k * throttle + m; // Star
//     z[17] = (float) k * throttle + m; // Port

//     // rudder
//     z[18] = 500.0f * vx * vx * Arudder * (((float)(rollrudder - 1518)*(180.0f/410.0f))*((float)(rollrudder - 1518)*(180.0f/410.0f))*(-0.0018f) + 0.003f*((float)(rollrudder - 1518)*(180.0f/410.0f)) - 0.0314);
//     z[19] = ((float)(rollrudder - 1518)*(180.0f/410.0f)) * (-0.076f) * 500.0f * vx * vx * Arudder;

//     // main
//     z[20] = 500.0f * vx * vx * Awing * (((float)(mainWing - 1518)*(180.0f/410.0f))*((float)(mainWing - 1518)*(180.0f/410.0f))*(-0.0018f) + 0.003f*((float)(mainWing - 1518)*(180.0f/410.0f)) - 0.0314);
//     z[21] = ((float)(mainWing - 1518)*(180.0f/410.0f)) * (-0.076f) * 500.0f * vx * vx * Awing;

//     // elevator
//     z[22] = 500.0f * vx * vx * Aelevator * (((float)(elevator - 1518)*(180.0f/410.0f))*((float)(elevator - 1518)*(180.0f/410.0f))*(-0.0018f) + 0.003f*((float)(elevator - 1518)*(180.0f/410.0f)) - 0.0314);
//     z[23] = ((float)(elevator - 1518)*(180.0f/410.0f)) * (-0.076f) * 500.0f * vx * vx * Aelevator;

// }




// void send_data( float vx, float Z[24] ) 
// {

//     // Shift DEC comma to the right before converting to int
//     long z0  = (int32_t)( 1000000.0f * Z[0]  ) ;
//     long z1  = (int32_t)( 1000000.0f * vx    ) ;
//     long z2  = (int32_t)( 1000000.0f * Z[2]  ) ;
//     long z3  = (int32_t)( 1000000.0f * Z[3]  ) ;
//     long z4  = (int32_t)( 1000000.0f * Z[4]  ) ;
//     long z5  = (int32_t)( 1000000.0f * Z[5]  ) ;
//     long z6  = (int32_t)( 1000000.0f * Z[6]  ) ;
//     long z7  = (int32_t)( 1000000.0f * Z[7]  ) ;
//     long z8  = (int32_t)( 1000000.0f * Z[8]  ) ;
//     long z9  = (int32_t)( 1000000.0f * Z[9]  ) ;
//     long z10 = (int32_t)( 1000000.0f * Z[10] ) ;
//     long z11 = (int32_t)( 1000000.0f * Z[11] ) ;

//     // Pack values to messages
//     msg200.buf[0] = z0 >> 24;
//     msg200.buf[1] = z0 >> 16;
//     msg200.buf[2] = z0 >> 8;
//     msg200.buf[3] = z0;
//     msg200.buf[4] = z1 >> 24;
//     msg200.buf[5] = z1 >> 16;
//     msg200.buf[6] = z1 >> 8;
//     msg200.buf[7] = z1;

//     msg201.buf[0] = z2 >> 24;
//     msg201.buf[1] = z2 >> 16;
//     msg201.buf[2] = z2 >> 8;
//     msg201.buf[3] = z2;
//     msg201.buf[4] = z3 >> 24;
//     msg201.buf[5] = z3 >> 16;
//     msg201.buf[6] = z3 >> 8;
//     msg201.buf[7] = z3;

//     msg202.buf[0] = z4 >> 24;
//     msg202.buf[1] = z4 >> 16;
//     msg202.buf[2] = z4 >> 8;
//     msg202.buf[3] = z4;
//     msg202.buf[4] = z5 >> 24;
//     msg202.buf[5] = z5 >> 16;
//     msg202.buf[6] = z5 >> 8;
//     msg202.buf[7] = z5;

//     msg203.buf[0] = z6 >> 24;
//     msg203.buf[1] = z6 >> 16;
//     msg203.buf[2] = z6 >> 8;
//     msg203.buf[3] = z6;
//     msg203.buf[4] = z7 >> 24;
//     msg203.buf[5] = z7 >> 16;
//     msg203.buf[6] = z7 >> 8;
//     msg203.buf[7] = z7;

//     msg204.buf[0] = z8 >> 24;
//     msg204.buf[1] = z8 >> 16;
//     msg204.buf[2] = z8 >> 8;
//     msg204.buf[3] = z8;
//     msg204.buf[4] = z9 >> 24;
//     msg204.buf[5] = z9 >> 16;
//     msg204.buf[6] = z9 >> 8;
//     msg204.buf[7] = z9;

//     msg205.buf[0] = z10 >> 24;
//     msg205.buf[1] = z10 >> 16;
//     msg205.buf[2] = z10 >> 8;
//     msg205.buf[3] = z10;
//     msg205.buf[4] = z11 >> 24;
//     msg205.buf[5] = z11 >> 16;
//     msg205.buf[6] = z11 >> 8;
//     msg205.buf[7] = z11;


//     // Write messages to CAN-bus
//     Can1.write(msg200);
//     Can1.write(msg201);
//     Can1.write(msg202);
//     Can1.write(msg203);
//     Can1.write(msg204);
//     Can1.write(msg205);
// }


// void send_one()
// {

//     // Shift DEC comma to the right before converting to int
//     long z0  = (int32_t)( 1000000.0f * 10.0f  ) ;
//     long z1  = (int32_t)( 1000000.0f * 11.0f  ) ;
//     long z2  = (int32_t)( 1000000.0f * 12.0f  ) ;
//     long z3  = (int32_t)( 1000000.0f * 13.0f  ) ;
//     long z4  = (int32_t)( 1000000.0f * 14.0f  ) ;
//     long z5  = (int32_t)( 1000000.0f * 15.0f  ) ;
//     long z6  = (int32_t)( 1000000.0f * 16.0f  ) ;
//     long z7  = (int32_t)( 1000000.0f * 17.0f  ) ;
//     long z8  = (int32_t)( 1000000.0f * 18.0f  ) ;
//     long z9  = (int32_t)( 1000000.0f * 19.0f  ) ;
//     long z10 = (int32_t)( 1000000.0f * 20.0f  ) ;
//     long z11 = (int32_t)( 1000000.0f * 21.0f  ) ;

//     // Pack values to messages
//     msg200.buf[0] = z0 >> 24;
//     msg200.buf[1] = z0 >> 16;
//     msg200.buf[2] = z0 >> 8;
//     msg200.buf[3] = z0;
//     msg200.buf[4] = z1 >> 24;
//     msg200.buf[5] = z1 >> 16;
//     msg200.buf[6] = z1 >> 8;
//     msg200.buf[7] = z1;

//     msg201.buf[0] = z2 >> 24;
//     msg201.buf[1] = z2 >> 16;
//     msg201.buf[2] = z2 >> 8;
//     msg201.buf[3] = z2;
//     msg201.buf[4] = z3 >> 24;
//     msg201.buf[5] = z3 >> 16;
//     msg201.buf[6] = z3 >> 8;
//     msg201.buf[7] = z3;

//     msg202.buf[0] = z4 >> 24;
//     msg202.buf[1] = z4 >> 16;
//     msg202.buf[2] = z4 >> 8;
//     msg202.buf[3] = z4;
//     msg202.buf[4] = z5 >> 24;
//     msg202.buf[5] = z5 >> 16;
//     msg202.buf[6] = z5 >> 8;
//     msg202.buf[7] = z5;

//     msg203.buf[0] = z6 >> 24;
//     msg203.buf[1] = z6 >> 16;
//     msg203.buf[2] = z6 >> 8;
//     msg203.buf[3] = z6;
//     msg203.buf[4] = z7 >> 24;
//     msg203.buf[5] = z7 >> 16;
//     msg203.buf[6] = z7 >> 8;
//     msg203.buf[7] = z7;

//     msg204.buf[0] = z8 >> 24;
//     msg204.buf[1] = z8 >> 16;
//     msg204.buf[2] = z8 >> 8;
//     msg204.buf[3] = z8;
//     msg204.buf[4] = z9 >> 24;
//     msg204.buf[5] = z9 >> 16;
//     msg204.buf[6] = z9 >> 8;
//     msg204.buf[7] = z9;

//     msg205.buf[0] = z10 >> 24;
//     msg205.buf[1] = z10 >> 16;
//     msg205.buf[2] = z10 >> 8;
//     msg205.buf[3] = z10;
//     msg205.buf[4] = z11 >> 24;
//     msg205.buf[5] = z11 >> 16;
//     msg205.buf[6] = z11 >> 8;
//     msg205.buf[7] = z11;


//     // Write messages to CAN-bus
//     Can1.write(msg200);
//     Can1.write(msg201);
//     Can1.write(msg202);
//     Can1.write(msg203);
//     Can1.write(msg204);
//     Can1.write(msg205);

//     Serial.println("BBBBBBBBBBBBAAAAAAAAAAAAAGGGGGGQQQQ");    

// }