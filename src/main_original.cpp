// #include <Arduino.h>
// #include "KalmanFilter.h"
// #include <iostream>

// #include "SensorFusion.h"     // Mahony filter
// // #include "can_comm.h"         // CAN communication

// KalmanFilter kf;              // EKF object
// Eigen::MatrixXf x(NS, 1);     // State vector
// Eigen::MatrixXf u(NC, 1);     // Control vector
// Eigen::MatrixXf z(NM, 1);     // Measurement vector

// SF mahony;                    // Mahony filter object
// float dt;                     // This is used for the Mahony filter, but what is it?
// float roll, pitch, yaw;       // Euler angles

// uint8_t datagram_type = 0x93; // Rate, accel, and inclinometer datagram type (See Table 5.21 in STIM300 datasheet)
// const int datagram_size = 38; // Size of the datagram
// uint8_t in_byte = 0;
// int datagram_size_count = 0;
// bool new_datagram = false;
// uint8_t datagram_raw[datagram_size]; // Datagram container array

// double acc_x = 0.0;
// double acc_y = 0.0;
// double acc_z = 0.0;
// double gyr_x = 0.0;
// double gyr_y = 0.0;
// double gyr_z = 0.0;
// double inc_x = 0.0;
// double inc_y = 0.0;
// double inc_z = 0.0;
// double g = 9.80665;
// double pi = 3.14159265359;


// ////////////////////////////////////////////////////////////////////
// //                            Sensors                             //
// ////////////////////////////////////////////////////////////////////
// void readDatagram()
// {
//     /*
//     Each datagram contains 38 bytes of data, the 0th byte is always 0x93, 10th, 20th, and 30th bytes are always 0x00
//     We can use this information to determine the start and end of a datagram so that we can read the data in groups of 38 bytes.
//     */

//     while (true)
//     {
//         if (Serial1.available())
//         {
//             in_byte = Serial1.read();

//             if (in_byte == datagram_type)
//             {
//                 datagram_size_count = 0;
//                 // new_datagram = true;
//                 datagram_raw[0] = in_byte;

//                 // Read the next 37 bytes of the datagram from Serial1 and store them in the remaining spots of the datagram_raw array
//                 Serial1.readBytes(&(datagram_raw[1]), datagram_size - 1);

//                 // Check if measurement status is OK: 10->Gyro, 20->Accel, 30->Incl
//                 if (datagram_raw[10] == 0x00 && datagram_raw[20] == 0x00 && datagram_raw[30] == 0x00)
//                     break;
//             }
//         }
//     }
// }

// double twosCompToDec(uint8_t msb, uint8_t mid, uint8_t lsb) 
// {
//     if (msb & ((uint8_t)128)) 
//         return (signed int)-(((msb^255)<<16) + ((mid^255)<<8) + (lsb^255));
//     else
//         return (signed int)((msb<<16) + (mid<<8) + (lsb));
// }

// void parseDatagram()
// {
//     gyr_x = twosCompToDec(datagram_raw[1], datagram_raw[2], datagram_raw[3]) / (2<<13);
//     gyr_y = twosCompToDec(datagram_raw[4], datagram_raw[5], datagram_raw[6]) / (2<<13);
//     gyr_z = twosCompToDec(datagram_raw[7], datagram_raw[8], datagram_raw[9]) / (2<<13);    
//     acc_x = twosCompToDec(datagram_raw[11], datagram_raw[12], datagram_raw[13]) / (2<<18);
//     acc_y = twosCompToDec(datagram_raw[14], datagram_raw[15], datagram_raw[16]) / (2<<18);
//     acc_z = twosCompToDec(datagram_raw[17], datagram_raw[18], datagram_raw[19]) / (2<<18);     
//     inc_x = twosCompToDec(datagram_raw[21], datagram_raw[22], datagram_raw[23]) / (2<<21);
//     inc_y = twosCompToDec(datagram_raw[24], datagram_raw[25], datagram_raw[26]) / (2<<21);
//     inc_z = twosCompToDec(datagram_raw[27], datagram_raw[28], datagram_raw[29]) / (2<<21);

//     gyr_x = gyr_x * pi / 180;
//     gyr_y = gyr_y * pi / 180;
//     gyr_z = gyr_z * pi / 180;
//     acc_x = acc_x * g;
//     acc_y = acc_y * g;
//     acc_z = acc_z * g;
// }

// void readSensors()
// {
//     Serial.println("AAAAAA");
//     readDatagram();
//     Serial.println("BBBBBB");
//     // parseDatagram();
// }

// void mahony_filter()
// {

//   // Mahony Filter: Obtain orientation
//   dt = mahony.deltatUpdate();
//   mahony.MahonyUpdate(gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z, dt);
//   pitch = mahony.getPitchRadians(); roll = mahony.getRollRadians(); yaw = mahony.getYawRadians();
// }

// ////////////////////////////////////////////////////////////////////
// //                           Printing                             //
// ////////////////////////////////////////////////////////////////////
// void print_state(Eigen::MatrixXf x)
// {
//   // print out the state on the serial monitor
//   Serial.print("| ");
//   Serial.print("x = ");
//   Serial.print(x(0, 0));
//   Serial.print(" | ");
//   Serial.print("y = ");
//   Serial.print(x(1, 0));
//   Serial.print(" | ");
//   Serial.print("z = ");
//   Serial.print(x(2, 0));
//   Serial.print(" | ");
//   Serial.print("phi = ");
//   Serial.print(x(3, 0));
//   Serial.print(" | ");
//   Serial.print("theta = ");
//   Serial.print(x(4, 0));
//   Serial.print(" | ");
//   Serial.print("psi = ");
//   Serial.print(x(5, 0));
//   Serial.print(" | ");
//   Serial.print("vx = ");
//   Serial.print(x(6, 0));
//   Serial.print(" | ");
//   Serial.print("vy = ");
//   Serial.print(x(7, 0));
//   Serial.print(" | ");
//   Serial.print("vz = ");
//   Serial.print(x(8, 0));
//   Serial.print(" | ");
//   Serial.print("wx = ");
//   Serial.print(x(9, 0));
//   Serial.print(" | ");
//   Serial.print("wy = ");
//   Serial.print(x(10, 0));
//   Serial.print(" | ");
//   Serial.print("wz = ");
//   Serial.print(x(11, 0));
//   Serial.println(" | ");

// }


// void setup()
// {

//   Serial.begin(115200);
//   Serial1.begin(921600);  // Serial port for IMU

//   u <<    113.1111,   // m1 [N]
//           113.1111,   // m2 [N]
//           0.0291,     // main wing [rad]
//           0.0,        // rudder wing [rad]
//           -0.0607;    // elevator wing [rad]

//   z << -0.5, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0;

//   x = kf.update(u, z);

// //   // Initialize can bus
// //   canInit();

// }

// void loop()
// {
//   Serial.println("00000");
//   readSensors();  // Read sensors: Obtain linear acceleration, angular velocity, (TODO: inclination)

//   // mahony_filter();  // Mahony filter: Obtain roll, pitch, yaw
//   // // z << z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz;
//   // z << -0.5, roll, pitch, yaw, 6.0, 0.0, 0.0, gyr_x, gyr_y, gyr_z;
//   // x = kf.update(u, z);  // EKF

//   // print_state(x);

// //   send_one();

// }


