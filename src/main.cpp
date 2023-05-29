#include <Arduino.h>
#include "KalmanFilter.h"
#include <iostream>

#include "SensorFusion.h"     // Mahony filter
#include "can_comm.h"         // CAN communication
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// CAN bus
std::vector<float> state_CAN(NS, 0.0);  // This vector will be used to send the estimated state by the EKF on the CAN bus

// EKF
KalmanFilter kf;              // EKF object
Eigen::MatrixXf x(NS, 1);     // State vector
Eigen::MatrixXf u(NC, 1);     // Control vector
Eigen::MatrixXf z(NM, 1);     // Measurement vector
// GNSS
SFE_UBLOX_GNSS myGNSS;        // GNSS object
long ground_speed = 0;        // Ground speed in mm/s
float groundSpeed = 0;        // Ground speed in m/s
// Mahony filter
SF mahony;                    // Mahony filter object
float dt;                     // This is used for the Mahony filter, but what is it?
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;
// IMU
uint8_t datagram_type = 0x93; // Rate, accel, and inclinometer datagram type (See Table 5.21 in STIM300 datasheet)
const int datagram_size = 38; // Size of the datagram
uint8_t in_byte = 0;
int datagram_size_count = 0;
bool new_datagram = false;
uint8_t datagram_raw[datagram_size]; // Datagram container array



// Global variables
float *fore_alt = new float(0.0);
float *aft_alt = new float(0.0);
float *wing_angle = new float(0.0);
float *rudder_angle = new float(0.0);
float *elevator_angle = new float(0.0);
float *throttle = new float(0.0);

double vel_x = 0.0;
double vel_y = 0.0;
double vel_z = 0.0;
double acc_x = 0.0;
double acc_y = 0.0;
double acc_z = 0.0;
double gyr_x = 0.0;
double gyr_y = 0.0;
double gyr_z = 0.0;
double inc_x = 0.0;
double inc_y = 0.0;
double inc_z = 0.0;
double g = 9.80665;
double pi = 3.14159265359;


////////////////////////////////////////////////////////////////////
//                         Initialization                         //
////////////////////////////////////////////////////////////////////
void initGNSS()
{
  Wire.begin();

  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
}

////////////////////////////////////////////////////////////////////
//                            Sensors                             //
////////////////////////////////////////////////////////////////////
void readDatagram()
{
    /*
    Each datagram contains 38 bytes of data, the 0th byte is always 0x93, 10th, 20th, and 30th bytes are always 0x00
    We can use this information to determine the start and end of a datagram so that we can read the data in groups of 38 bytes.
    */

    while (true)
    {
        if (Serial1.available())
        {
            in_byte = Serial1.read();

            if (in_byte == datagram_type)
            {
                datagram_size_count = 0;
                // new_datagram = true;
                datagram_raw[0] = in_byte;

                // Read the next 37 bytes of the datagram from Serial1 and store them in the remaining spots of the datagram_raw array
                Serial1.readBytes(&(datagram_raw[1]), datagram_size - 1);

                // Check if measurement status is OK: 10->Gyro, 20->Accel, 30->Incl
                if (datagram_raw[10] == 0x00 && datagram_raw[20] == 0x00 && datagram_raw[30] == 0x00)
                    break;
            }
        }
    }
}

double twosCompToDec(uint8_t msb, uint8_t mid, uint8_t lsb) 
{
    if (msb & ((uint8_t)128)) 
        return (signed int)-(((msb^255)<<16) + ((mid^255)<<8) + (lsb^255));
    else
        return (signed int)((msb<<16) + (mid<<8) + (lsb));
}

void parseDatagram()
{
    gyr_x = twosCompToDec(datagram_raw[1], datagram_raw[2], datagram_raw[3]) / (2<<13);
    gyr_y = twosCompToDec(datagram_raw[4], datagram_raw[5], datagram_raw[6]) / (2<<13);
    gyr_z = twosCompToDec(datagram_raw[7], datagram_raw[8], datagram_raw[9]) / (2<<13);    
    acc_x = twosCompToDec(datagram_raw[11], datagram_raw[12], datagram_raw[13]) / (2<<18);
    acc_y = twosCompToDec(datagram_raw[14], datagram_raw[15], datagram_raw[16]) / (2<<18);
    acc_z = twosCompToDec(datagram_raw[17], datagram_raw[18], datagram_raw[19]) / (2<<18);     
    inc_x = twosCompToDec(datagram_raw[21], datagram_raw[22], datagram_raw[23]) / (2<<21);
    inc_y = twosCompToDec(datagram_raw[24], datagram_raw[25], datagram_raw[26]) / (2<<21);
    inc_z = twosCompToDec(datagram_raw[27], datagram_raw[28], datagram_raw[29]) / (2<<21);

    gyr_x = gyr_x * pi / 180;
    gyr_y = gyr_y * pi / 180;
    gyr_z = gyr_z * pi / 180;
    acc_x = acc_x * g;
    acc_y = acc_y * g;
    acc_z = acc_z * g;
}

void readIMU()
{
    readDatagram();
    parseDatagram();
}

void mahonyFilter()
{

  // Mahony Filter: Obtain orientation
  dt = mahony.deltatUpdate();
  mahony.MahonyUpdate(gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z, dt);
  pitch = mahony.getPitchRadians(); roll = mahony.getRollRadians(); yaw = mahony.getYawRadians();
}

void readGPS()
{
  ground_speed = myGNSS.getGroundSpeed();                   // Ground speed in mm/s
  groundSpeed = static_cast<float>(ground_speed) / 1000.0;  // Convert to m/s

  // We assume that vy = 0.0, vz = 0.0, vx = groundSpeed (Adjust accordingly to your needs)
  vel_x = groundSpeed;

}

void readSensors()
{
    readIMU();        // angular velocity, linear acceleration, and inclination
    mahonyFilter();   // orientation
    readGPS();        // Ground speed, position(Latitude, Longitude)
}


////////////////////////////////////////////////////////////////////
//                           Printing                             //
////////////////////////////////////////////////////////////////////
void print_state(Eigen::MatrixXf x)
{
  // print out the state on the serial monitor
  Serial.print("| ");
  Serial.print("x = ");
  Serial.print(x(0, 0));
  Serial.print(" | ");
  Serial.print("y = ");
  Serial.print(x(1, 0));
  Serial.print(" | ");
  Serial.print("z = ");
  Serial.print(x(2, 0));
  Serial.print(" | ");
  Serial.print("phi = ");
  Serial.print(x(3, 0));
  Serial.print(" | ");
  Serial.print("theta = ");
  Serial.print(x(4, 0));
  Serial.print(" | ");
  Serial.print("psi = ");
  Serial.print(x(5, 0));
  Serial.print(" | ");
  Serial.print("vx = ");
  Serial.print(x(6, 0));
  Serial.print(" | ");
  Serial.print("vy = ");
  Serial.print(x(7, 0));
  Serial.print(" | ");
  Serial.print("vz = ");
  Serial.print(x(8, 0));
  Serial.print(" | ");
  Serial.print("wx = ");
  Serial.print(x(9, 0));
  Serial.print(" | ");
  Serial.print("wy = ");
  Serial.print(x(10, 0));
  Serial.print(" | ");
  Serial.print("wz = ");
  Serial.print(x(11, 0));
  Serial.println(" | ");

}


void setup()
{
  delay(1000);
  Serial.begin(115200);   // Serial port for debugging
  Serial1.begin(921600);  // Serial port for IMU
  initGNSS();            // Setup GNSS
  init_can();             // Setup CAN    // From the can_comm.h library


  u <<    113.1111,   // m1 [N]
          113.1111,   // m2 [N]
          0.0291,     // main wing [rad]
          0.0,        // rudder wing [rad]
          -0.0607;    // elevator wing [rad]

  z << -0.5,    // altitude [m]
        0.0,    // roll [rad]
        0.0,    // pitch [rad]
        0.0,    // yaw [rad]
        6.0,    // vx [m/s]
        0.0,    // vy [m/s]
        0.0,    // vz [m/s]
        0.0,    // wx [rad/s]
        0.0,    // wy [rad/s]
        0.0;    // wz [rad/s]

  x = kf.update(u, z);

}

void loop()
{
  read_can(fore_alt, aft_alt, wing_angle, rudder_angle, elevator_angle, throttle);

  readSensors();

  z << -0.5,    // altitude [m]
        roll,   // roll [rad]
        pitch,  // pitch [rad]
        yaw,    // yaw [rad]
        vel_x,  // vx [m/s]
        vel_y,  // vy [m/s]
        vel_z,  // vz [m/s]
        gyr_x,  // wx [rad/s]
        gyr_y,  // wy [rad/s]
        gyr_z;  // wz [rad/s]

  x = kf.update(u, z);  // EKF

  print_state(x);

  // Update the entries of the std::vector state_CAN by the entries of the state vector Eigen::MatrixXf x
  for (int i = 0; i < 12; i++)
  {
    state_CAN[i] = x(i, 0);
  }

  // Send the state vector to the CAN bus
  write_can(state_CAN);
  

}


