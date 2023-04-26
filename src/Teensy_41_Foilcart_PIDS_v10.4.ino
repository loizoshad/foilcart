// // -------------------------------------------------------------
// //Reads all traffic on Foilcart CAN0 and parses it to static variables, runs fundamental loop for processing and sends control signals back
// // First version by Nicholas Honeth <honeth@kth.se> December 2020
// //Updated 2021-08-30 added GPS SoG and ed pitch & roll scale factors 
// //Updated 2021-11-11 Use PID limits instead of scaling on Roll and Pitch outputs
// //Updated 2022-05-19 Added 3rd control pos roll setpoint regulation, new default values from tests 
// //Updated 2022-08-17 Controller01 and 02 are the same, except for yaw input coming straight from the remote on controller 02
// //Updated 2022-08-18 RollStabInput is now updated with euler_X each iteration (probably should've been this way from the start)
// // Example parameter update for pitchRateP: cansend can0 073#00000000

// #include <FlexCAN_T4.h>

// //#ifndef __MK66FX1M0__
// //  #error "Teensy 4.1 with dual CAN bus is required to run this example"
// //#endif

// // Constants:
// static float g    = 9.81;  // gravity
// static float pi   = 3.1416;
// static int loopPeriod = 5;// in ms

// static long elapsedTime;
// static unsigned long lastTime;

// static int pwmZero = 1518; // pwmZero - pwmSpan <= PWM <= pwmZero + pwmSpan
// static int pwmSpan = 420;

// // Remote control inputs:
// static int rmtThrottle = 1938; // min 1938 max 1098
// static int rmtRoll = pwmZero;
// static int rmtElevator = pwmZero;
// static int rmtYaw = pwmZero;
// static int rmtFlaps = pwmZero;
// static int rmtWingAngle = pwmZero;
// static int rmtThrottleDir = 1285; //Forward, stationary and reverse
// static int rmtCtrlType = 1100; //Allows mode switch between manual, stabilise and auto

// // Maximum remote impact on setpoints (in degrees)
// static int rmtRollMax = 15; 
// static int rmtPitchMax = 15; 
// static int rmtYawMax = 15;

// // Deg/s to give maximum actuation
// static int outRollRateMax = 15; 
// static int outPitchRateMax = 15; 
// static int outYawRateMax = 15; 

// // Motor temp sensors: TODO: Convert to degrees
// static int tempMotor1;
// static int tempMotor2;

// // IMU Eulers in rad
// static float euler_X = 0.0;
// static float euler_Y = 0.0;
// static float euler_Z = 0.0;

// static float turnRate;  // 

// // IMU rotational velocities in rad/s
// static float rot_X = 0.0;
// static float rot_Y = 0.0;
// static float rot_Z = 0.0;
// static float recv_rot_X = 0.0;
// static float recv_rot_Y = 0.0;

// // GPS SoG in m/s
// static float gpsSoG = 0.0;
// static long gpsTimeout = 0;

// // Senix height sensor values
// static int senixFore = 0; // [mm*10]
// static int senixAft = 0;
// static int AltForeOffset = 620; // Measureed keel line [mm]
// static int AltAftOffset = 360;

// // Control return values:
// static int ctrlThrottle         = 1938;   // min 1938 max 1098
// static int ctrlRoll             = pwmZero;
// static int ctrlElevator         = pwmZero;
// static int ctrlYaw              = pwmZero;
// static int ctrlFlaps            = pwmZero;
// static int ctrlWingAngle        = pwmZero;

// static float rollOffsetRad        = 0; //calculated value from user input to offset roll setpoints
// //static float rollStabSetpoint_CT  = 0; //For Coordinated turn

// // Control parameters

// static double turnRateSetpointPar = 0.0; // Used in coordinated turn

// static double rollStabInput     = 0.0;
// static double rollStabSetpoint  = 0.0;
// static double rollStabSetpoint_CT  = 0.0;
// static double rollStabOutput    = 0.0;
// static float rollStabP          = 1.0; //CAN79    // 20
// static float rollStabI          = 0.0;
// static float rollStabD          = 0.0;
// static double rollStabSetpointPar = 0.0; //Persistence of rollStabSetpoint

// static double rollRateInput     = 0.0;
// static double rollRateSetpoint  = 0.0;
// static double rollRateOutput    = 0.0;
// static float rollRateP          = 1.0; //CAN82  //1450
// static float rollRateI          = 0.0;
// static float rollRateD          = 0.0;

// static double pitchStabInput    = 0.0;
// static double pitchStabSetpoint = 0.0;
// static double pitchStabOutput   = 0.0;
// static double pitchStabP        = 1.0;    //0.4
// static double pitchStabI        = 0.0;
// static double pitchStabD        = 0.0;
// static double pitchStabSetpointPar = 0.0;

// static double pitchRateInput    = 0.0;
// static double pitchRateSetpoint = 0.0;
// static double pitchRateSetpoint_CT = 0.0;
// static double pitchRateOutput   = 0.0;
// static float pitchRateP         = 1.0;    //30
// static float pitchRateI         = 0.0;
// static float pitchRateD         = 0.0;
// static double pitchRateSetpointPar = 0.0; //Persistence of pitchRateSetpoint

// static float yawRateThresholdCompress = 1.4;
// static float yawRateSpeedLimit        = 4.0; //CAN59
// static float turnRateOffsetRad        = 0.0;
// static float turnRateSetpoint         = 0.0;

// static double yawStabInput      = 0.0;
// static double yawStabSetpoint   = 0.0;
// static double yawStabOutput     = 0.0;
// static float yawStabP           = 0.0;
// static float yawStabI           = 0.0;
// static float yawStabD           = 0.0;

// static double yawRateInput        = 0.0;
// static double yawRateSetpoint     = 0.0;
// static double yawRateSetpoint_CT  = 0.0;
// static double yawRateOutput       = 0.0;
// static float yawRateP             = 1.0;
// static float yawRateI             = 0.0;
// static float yawRateD             = 0.0;
// static double yawRateSetpointPar  = 0.0; //Persistence of yawRateSetpoint

// static unsigned altStabCalc     = 0;
// static double altStabInput      = 0.0;
// static double altStabSetpoint   = 200; //CAN63
// static double altStabOutput     = 0.0;
// static double altStabP          = 1/1000.0 * (pi/180) * 10; // Add 10 deg of pitch per meter in altitude error 
// static double altStabI          = 0.0;
// static double altStabD          = 0.0;

// static double velStabInput      = 0.0;
// static double velStabSetpoint   = 0.0;
// static double velStabOutput     = 0.0;
// static double velStabP          = 0.0;
// static double velStabI          = 0.0;
// static double velStabD          = 0.0;

// /* CAN Message schema
//  *  Inbound:
//  * 257 (0x101) Remote control ch 1-4
//  * 258 (0x102) Remote control ch 4-8 
//  *     (0x120) Senix altitude fore
//  *     (0x121) Senix altitude aft
//  *     (0x130) IMU Eulers X & Y
//  *     (0x131) IMU Eulers Z
//  *     (0x132) IMU rot X & Y
//  *     (0x133) IMU rot Z
//  *     (0x140) Temp motor 1
//  *     (0x141) Temp motor 2
//  *     
//  *  Outbound:
//  *     (0x105) Feedback control signals ch 1-4
//  *     (0x106) Feedback control signals ch 5-8
//  */

// FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
// FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;

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

// static CAN_message_t msg134; //Acc_XY
// static CAN_message_t msg135; // Acc_Z
// static CAN_message_t msg136; // Ang vel X&Y
// static CAN_message_t msg137; // Ang vel Z
// static CAN_message_t msg138; // Inc X&Y
// static CAN_message_t msg139; // Inc Z

// static CAN_message_t msg147; // Roll PID output values
// static CAN_message_t msg148; // Pitch PID output values
// static CAN_message_t msg149; // Yaw PID output values
// static CAN_message_t msg150; // Alt PID output values

// static uint8_t hex[17] = "0123456789abcdef";


// // -------------------------------------------------------------
// void clearTerminalScreen() {
//   Serial.write(27);       // ESC command
//   Serial.print("[2J");    // clear screen command
//   Serial.write(27);
//   Serial.print("[H");     // cursor to home command
// }

// // -------------------------------------------------------------
// void printValues() {
//      clearTerminalScreen();
//      Serial.println("Inbound CAN data values");
//      Serial.println("------------------------------------------");
//      Serial.print("rmtThrottle: ");
//      Serial.println(rmtThrottle);
//      Serial.print("rmtYaw: ");
//      Serial.println(rmtYaw);
//      Serial.print("rmtRoll: ");
//      Serial.println(rmtRoll);
//      Serial.print("rmtElevator: ");
//      Serial.println(rmtElevator);
//      Serial.print("rmtWingAngle: ");
//      Serial.println(rmtWingAngle);
//      Serial.print("rmtThrottleDir: ");
//      Serial.println(rmtThrottleDir);

//      Serial.println("---GPS------------------------------------");
//      Serial.print("GPS SoG: ");
//      Serial.println(gpsSoG);
     
//      Serial.println("---Alt------------------------------------");
//      // Senix height
//      Serial.print("senixFore: ");
//      Serial.println(senixFore);
//      Serial.print("senixAft: ");
//      Serial.println(senixAft);
     
     
//      Serial.println("---IMU------------------------------------");
     
     
//      //IMU cast to float and divided by 1,000,000:
//      Serial.print("euler_X: ");
//      Serial.println(((float)euler_X));
//      Serial.print("euler_Y: ");
//      Serial.println(((float)euler_Y));
//      Serial.print("euler_Z: ");
//      Serial.println(((float)euler_Z));
//      Serial.print("rot_X: ");
//      Serial.println(((float)rot_X));
//      Serial.print("rot_Y: ");
//      Serial.println(((float)rot_Y));
//      Serial.print("rot_Z: ");
//      Serial.println(((float)rot_Z));
     
//      Serial.println("---CAN-out--------------------------------");
//      Serial.println("Outbound CAN data values");
     
//      Serial.print("ctrlThrottle: ");
//      Serial.println(ctrlThrottle);
//      Serial.print("rmtYaw: ");
//      Serial.println(ctrlYaw);
//      Serial.print("ctrlRoll: ");
//      Serial.println(ctrlRoll);
//      Serial.print("ctrlElevator: ");
//      Serial.println(ctrlElevator);
//      Serial.print("ctrlWingAngle: ");
//      Serial.println(ctrlWingAngle);
     
// } //printValues

// // -------------------------------------------------------------
// void canInit() {

//   Can1.begin();
//   Can1.setBaudRate(250000);

// //--------------------------------------
//   // Acknowledgement
//   msg50.flags.extended = 0;
//   msg50.id = 0x50;
//   msg50.len = 8;
//   msg50.buf[0] = 100;
//   msg50.buf[1] = 0;
//   msg50.buf[2] = 0;
//   msg50.buf[3] = 0;
//   msg50.buf[4] = 0;
//   msg50.buf[5] = 0;
//   msg50.buf[6] = 0;
//   msg50.buf[7] = 0;

// //--------------------------------------
//   // RC inputs 1-4
//   msg101.flags.extended = 0;
//   msg101.id = 0x101;
//   msg101.len = 8;
//   msg101.buf[0] = (uint) 1100;
//   msg101.buf[1] = (uint) 1100;
//   msg101.buf[2] = (uint) 1100;
//   msg101.buf[3] = (uint) 1100;
//   msg101.buf[4] = (uint) 1100;
//   msg101.buf[5] = (uint) 1100;
//   msg101.buf[6] = (uint) 1100;
//   msg101.buf[7] = (uint) 1100;

// //--------------------------------------
//   // RC inputs 5-8
//   msg102.flags.extended = 0;
//   msg102.id = 0x102;
//   msg102.len = 8;
//   msg102.buf[0] = 1100;
//   msg102.buf[1] = 1100;
//   msg102.buf[2] = 1100;
//   msg102.buf[3] = 1100;
//   msg102.buf[4] = 1100;
//   msg102.buf[5] = 1100;
//   msg102.buf[6] = 1100;
//   msg102.buf[7] = 1100;

// //--------------------------------------
//   // Control outputs 1-4
//   msg105.flags.extended = 0;
//   msg105.id = 0x105;
//   msg105.len = 8;
//   msg105.buf[0] = 1100;
//   msg105.buf[1] = 1100;
//   msg105.buf[2] = 1100;
//   msg105.buf[3] = 1100;
//   msg105.buf[4] = 1100;
//   msg105.buf[5] = 1100;
//   msg105.buf[6] = 1100;
//   msg105.buf[7] = 1100;

// //--------------------------------------
//   // Control outputs 5-8
//   msg106.flags.extended = 0;
//   msg106.id = 0x106;
//   msg106.len = 8;
//   msg106.buf[0] = 1100;
//   msg106.buf[1] = 1100;
//   msg106.buf[2] = 1100;
//   msg106.buf[3] = 1100;
//   msg106.buf[4] = 1100;
//   msg106.buf[5] = 1100;
//   msg106.buf[6] = 1100;
//   msg106.buf[7] = 1100;


// //--------------------------------------
//   //Euler angles X & Y
//   msg130.flags.extended = 0;
//   msg130.id = 0x130;
//   msg130.len = 8;
//   msg130.buf[0] = 0;
//   msg130.buf[1] = 0;
//   msg130.buf[2] = 0;
//   msg130.buf[3] = 0;
//   msg130.buf[4] = 0;
//   msg130.buf[5] = 0;
//   msg130.buf[6] = 0;
//   msg130.buf[7] = 0;

// // Euler angle Z
//   msg131.flags.extended = 0;
//   msg131.id = 0x131;
//   msg131.len = 4;
//   msg131.buf[0] = 0;
//   msg131.buf[1] = 0;
//   msg131.buf[2] = 0;
//   msg131.buf[3] = 0;
//   //msg131.buf[4] = 0;
//   //msg131.buf[5] = 0;
//   //msg131.buf[6] = 0;
//   //msg131.buf[7] = 0;

// //angular velocity X & Y
//   msg132.flags.extended = 0;
//   msg132.id = 0x132;
//   msg132.len = 8;
//   msg132.buf[0] = 0;
//   msg132.buf[1] = 0;
//   msg132.buf[2] = 0;
//   msg132.buf[3] = 0;
//   msg132.buf[4] = 0;
//   msg132.buf[5] = 0;
//   msg132.buf[6] = 0;
//   msg132.buf[7] = 0;

// // angular velocity Z
//   msg133.flags.extended = 0;
//   msg133.id = 0x133;
//   msg133.len = 4;
//   msg133.buf[0] = 0;
//   msg133.buf[1] = 0;
//   msg133.buf[2] = 0;
//   msg133.buf[3] = 0;
//   //msg131.buf[4] = 0;
//   //msg131.buf[5] = 0;
//   //msg131.buf[6] = 0;
//   //msg131.buf[7] = 0; 
// //--------------------------------------
//   //Acc X & Y
//   msg134.flags.extended = 0;
//   msg134.id = 0x134;
//   msg134.len = 6;
//   msg134.buf[0] = 0;
//   msg134.buf[1] = 0;
//   msg134.buf[2] = 0;
//   msg134.buf[3] = 0;
//   msg134.buf[4] = 0;
//   msg134.buf[5] = 0;
//   //msg134.buf[6] = 0;
//   //msg134.buf[7] = 0;

//   // Acc Z
//   msg135.flags.extended = 0;
//   msg135.id = 0x135;
//   msg135.len = 3;
//   msg135.buf[0] = 0;
//   msg135.buf[1] = 0;
//   msg135.buf[2] = 0;
//   //msg135.buf[3] = 0;          // ska inte den här vara bortkommaterad??
//   //msg135.buf[4] = 0;
//   //msg135.buf[5] = 0;
//   //msg135.buf[6] = 0;
//   //msg135.buf[7] = 0;

//   //angular velocity X & Y
//   msg136.flags.extended = 0;
//   msg136.id = 0x136;
//   msg136.len = 6;
//   msg136.buf[0] = 0;
//   msg136.buf[1] = 0;
//   msg136.buf[2] = 0;
//   msg136.buf[3] = 0;
//   msg136.buf[4] = 0;
//   msg136.buf[5] = 0;
//   //msg136.buf[6] = 0;
//   //msg136.buf[7] = 0;

//   // angular velocity Z
//   msg137.flags.extended = 0;
//   msg137.id = 0x137;
//   msg137.len = 3;
//   msg137.buf[0] = 0;
//   msg137.buf[1] = 0;
//   msg137.buf[2] = 0;
//   //msg137.buf[3] = 0;      // ska inte den här vara bortkommaterad??
//   //msg137.buf[4] = 0;
//   //msg137.buf[5] = 0;
//   //msg137.buf[6] = 0;
//   //msg137.buf[7] = 0;

//   //Inc X & Y
//   msg138.flags.extended = 0;
//   msg138.id = 0x138;
//   msg138.len = 6;
//   msg138.buf[0] = 0;
//   msg138.buf[1] = 0;
//   msg138.buf[2] = 0;
//   msg138.buf[3] = 0;
//   msg138.buf[4] = 0;
//   msg138.buf[5] = 0;
//   //msg138.buf[6] = 0;
//   //msg138.buf[7] = 0;

//   // Inc Z
//   msg139.flags.extended = 0;
//   msg139.id = 0x139;
//   msg139.len = 3;
//   msg139.buf[0] = 0;
//   msg139.buf[1] = 0;
//   msg139.buf[2] = 0;
//   //msg139.buf[3] = 0;        
//   //msg139.buf[4] = 0;
//   //msg139.buf[5] = 0;
//   //msg139.buf[6] = 0;
//   //msg139.buf[7] = 0;

  
// }  //CANinit


// // -------------------------------------------------------------
// static void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
// {
//   uint8_t working;
//   while( dumpLen-- ) {
//     working = *bytePtr++;
//     Serial.write( hex[ working>>4 ] );
//     Serial.write( hex[ working&15 ] );
//   }
//   Serial.write('\r');
//   Serial.write('\n');
// }


// // -------------------------------------------------------------
// void setup(void)
// {
//   delay(1000);
//   Serial.begin(115200);
//   Serial.println(F("Teensy 3.6 CAN PID stabilisation controller."));
//   Serial.println( "Compiled: " __FILE__ ", " __DATE__ ", " __TIME__ ", " __VERSION__);

//   Can1.begin();
//   Can1.setBaudRate(250000);  
//   Can2.begin();
//   Can2.setBaudRate(250000);

//   //if using enable pins on a transceiver they need to be set on
//   pinMode(2, OUTPUT);
//   pinMode(35, OUTPUT);

//   digitalWrite(2, HIGH);
//   digitalWrite(35, HIGH);

//   msg.flags.extended = 0;
//   msg.id = 0x100;
//   msg.len = 2;
//   msg.buf[0] = 10;
//   msg.buf[1] = 20;
//   //msg.buf[2] = 0;
//   //msg.buf[3] = 100;
//   //msg.buf[4] = 128;
//   //msg.buf[5] = 64;
//   //msg.buf[6] = 32;
//   //msg.buf[7] = 16;

//   canInit();

//   lastTime = micros();
//   //TODO: Add defualt values for variables
// }



// // -------------------------------------------------------------
// void sendAcknowledgement(int srcAdr, int insAdr, long insVal)
// {
//   // Acknowledgement
//   msg50.flags.extended = 0;
//   msg50.id = 0x50;
//   msg50.len = 8;
//   msg50.buf[0] = highByte(srcAdr);
//   msg50.buf[1] = lowByte(srcAdr);
//   msg50.buf[2] = highByte(insAdr);
//   msg50.buf[3] = lowByte(insAdr);
//   msg50.buf[4] = insVal >> 24;
//   msg50.buf[5] = insVal >> 16;
//   msg50.buf[6] = insVal >> 8;
//   msg50.buf[7] = insVal;
//   Can1.write(msg50);
// }

// void sendPIDVals() 
// {
//   long rollStabOutput_L = (long) (0.00001*1000000);
//   long rollRateOutput_L = (long) (0.00002*1000000);
//   long pitchStabOutput_L = (long) (0.00003*1000000);
//   long pitchRateOutput_L = (long) (0.00004*1000000);
//   long yawRateOutput_L = (long) (0.00005*1000000);
//   long altStabInput_L = (long) (0.00006*1000000);
//   long altStabOutput_L = (long) (0.00007*1000000);

//   msg147.id = 0x147; // Roll and rollrate PID outputs
//   msg147.buf[0] = rollStabOutput_L >> 24;
//   msg147.buf[1] = rollStabOutput_L >> 16;
//   msg147.buf[2] = rollStabOutput_L >> 8;
//   msg147.buf[3] = rollStabOutput_L;
//   msg147.buf[4] = rollRateOutput_L >> 24;
//   msg147.buf[5] = rollRateOutput_L >> 16;
//   msg147.buf[6] = rollRateOutput_L >> 8;
//   msg147.buf[7] = rollRateOutput_L;
//   Can1.write(msg147);

//   msg148.id = 0x148; // Pitch and pitchrate PID outputs
//   msg148.buf[0] = pitchStabOutput_L >> 24;
//   msg148.buf[1] = pitchStabOutput_L >> 16;
//   msg148.buf[2] = pitchStabOutput_L >> 8;
//   msg148.buf[3] = pitchStabOutput_L;
//   msg148.buf[4] = pitchRateOutput_L >> 24;
//   msg148.buf[5] = pitchRateOutput_L >> 16;
//   msg148.buf[6] = pitchRateOutput_L >> 8;
//   msg148.buf[7] = pitchRateOutput_L;
//   Can1.write(msg148);

//   msg149.id = 0x149; // Yawrate PID output
//   msg149.buf[0] = yawRateOutput_L >> 24;
//   msg149.buf[1] = yawRateOutput_L >> 16;
//   msg149.buf[2] = yawRateOutput_L >> 8;
//   msg149.buf[3] = yawRateOutput_L;
//   msg149.buf[4] = 0;
//   msg149.buf[5] = 0;
//   msg149.buf[6] = 0;
//   msg149.buf[7] = 0;
//   Can1.write(msg149);
  
//   msg150.id = 0x150; // AltStab in- and output
//   msg150.buf[0] = altStabInput_L >> 24;
//   msg150.buf[1] = altStabInput_L >> 16;
//   msg150.buf[2] = altStabInput_L >> 8;
//   msg150.buf[3] = altStabInput_L;
//   msg150.buf[4] = altStabOutput_L >> 24;
//   msg150.buf[5] = altStabOutput_L >> 16;
//   msg150.buf[6] = altStabOutput_L >> 8;
//   msg150.buf[7] = altStabOutput_L;
//   Can1.write(msg150);
// }


// // Converts a pwm value from range [1518 +/- 420] to an angle of [+/- angDeg], but expressed in radians (or rad/s)
// float pwm2rad(int pwmVal, int angDeg)
// {
//   return (float)(pwmVal - pwmZero) / pwmSpan * angDeg * pi / 180;
// }

// //  Converts (and constrains) a value in radians (or rad/s) from range [0 +/- angDeg] degrees (or degrees/s) to a pwm value in the range [1098,1938]
// int rad2pwm(float angRad, int angDegLim)
// {
//   float angDeg = constrain(angRad/pi*180, -angDegLim, angDegLim);
//   return (int) pwmZero + angDeg / angDegLim * pwmSpan;
// }

// // -------------------------------------------------------------
// float yawRateOutputScaling(float yawRateSpeedLimitLoc, float yawRateThresholdCompress, float gpsSoGLoc)
// { // Stays at 1 at velocities up to right below yawRateSpeedLimitLoc, then fades quickly to zero
//   float yawRateScale = tanh((yawRateSpeedLimitLoc - gpsSoGLoc) * yawRateThresholdCompress) * 0.5 + 0.5;
//   return yawRateScale;
// }



// // -------------------------------------------------------------
// void loop(void)
// {

//   //Read CAN messages
//   CAN_message_t inMsg;
//   while ((Can1.read(inMsg)!=0) && (millis() - lastTime > loopPeriod)) 
//   {
    
//     //Serial.print("CAN bus 1: "); hexDump(8, inMsg.buf);
//     switch(inMsg.id) {
//       case 0x99: // GPS Speed over Ground
//         //Serial.print("SoG: ");
//         gpsSoG = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         //Serial.println(gpsSoG);
//         break;
//       case 0x101: // RC Ch1-4
//         //Serial.println("RC Ch1-4");
//         rmtRoll = (int) word(inMsg.buf[0],inMsg.buf[1]);           //Ch1 Roll rudder
//         rmtElevator = (int) word(inMsg.buf[2],inMsg.buf[3]);       //Ch2 Elevator
//         rmtThrottle = (int) word(inMsg.buf[4],inMsg.buf[5]);       //Ch3
//         rmtYaw = (int) word(inMsg.buf[6],inMsg.buf[7]);            //Ch4 //To be set to thrust vector in motor control          
//         break;
        
//       case 0x102: // RC Ch5-8
//         //Serial.println("RC Ch5-8");
//         rmtWingAngle = word(inMsg.buf[0],inMsg.buf[1]);           //Ch5 //RD on control
//         rmtFlaps = word(inMsg.buf[2],inMsg.buf[3]);               //Ch6 // LD on control
//         rmtCtrlType = word(inMsg.buf[4],inMsg.buf[5]);            //Ch7  TODO: Set channel for manual, stabilize, autonomous
//         rmtThrottleDir = word(inMsg.buf[6],inMsg.buf[7]);         //Motor direction switch
//         break;
//       case 0x120: // Senix Alt fore
//         senixFore = word(inMsg.buf[0],inMsg.buf[1]);
//         break;
//       case 0x121: // Senix alt aft
//         senixAft = word(inMsg.buf[0], inMsg.buf[1]);
//         break;  
//       case 0x130: // Eulers X&Y
//         //Serial.println("Eulers X&Y");
//         euler_X = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         euler_Y = (float) ((inMsg.buf[4]<<24) | (inMsg.buf[5]<<16) | (inMsg.buf[6]<<8) | (inMsg.buf[7]))/1000000;
//         break;
//       case 0x131: // Euler Z
//         //Serial.println("Euler Z");
//         euler_Z = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         break;
//       case 0x132: // Rot X&Y
//         //Serial.println("Rot X&Y");
//         recv_rot_X = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         recv_rot_Y = (float) ((inMsg.buf[4]<<24) | (inMsg.buf[5]<<16) | (inMsg.buf[6]<<8) | (inMsg.buf[7]))/1000000;
//         if (abs(recv_rot_X) < 4) {
//           rot_X = recv_rot_X;
//         }
//         if (abs(recv_rot_Y) < 4) {
//           rot_Y = recv_rot_Y;         
//         }
//         break;
//       case 0x133: // Rot Z
//         //Serial.println("Rot Z");
//         rot_Z = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         break;
//       case 0x134: // Acc X&Y
//         //Serial.println("Acc X&Y");
//         break;
//       case 0x135: // Acc Z
//         //Serial.println("Acc Z");
//         break;
//       case 0x136: // Ang vel X&Y
//         //Serial.println("Ang vel X&Y");
//         break;
//       case 0x137: // Ang vel Z
//         //Serial.println("Ang vel Z");
//         break;
//       case 0x138: // Inc X&Y
//         //Serial.println("Inc X&Y");
//         break;
//       case 0x139: // Inc Z
//         //Serial.println("Inc Z");
//         break;
//       case 0x140: // Temp port
//         //Serial.println("Temp port");
//         break;
//       case 0x141: // Temp starb
//         //Serial.println("Temp starb");
//         break;
//       case 0x50: //Acknowledgement
//         //Serial.println("Incoming ack");
//         break;
// //      case 0x91: //Speed P
// //        //Serial.println("Speed P");
// //        break;
// //      case 0x92: //Speed I
// //        //Serial.println("Speed I");
// //        break;
// //      case 0x93: //Speed D
// //        //Serial.println("Speed D");
// //        break;
//       // ------------------------------------------------- SETPOINTS ---------------------------------------------------
      
//       case 0x58: // YawRateThresholdCompress 
//         Serial.println("YawRateThresholdCompress");
//         yawRateThresholdCompress = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x58, (long) (yawRateThresholdCompress*1000000));
//         break;
//       case 0x59: // YawRateSpeedLimit  
//         Serial.println("yawRateSpeedLimit");
//         yawRateSpeedLimit = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x59, (long) (yawRateSpeedLimit*1000000));
//         break;

//       case 0x60: // PitchStabSetpoint 
//         Serial.println("Pitch StabSetpoint");
//         pitchStabSetpoint = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x60, (long) (pitchStabSetpoint*1000000));
//         break;
//       case 0x61: // RollStabSetpoint  
//         Serial.println("Roll StabSetpoint");
//         rollStabSetpoint = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x61, (long) (rollStabSetpoint*1000000));
//         break;
//       case 0x62: // YawStabSetpoint  
//         Serial.println("Yaw StabSetpoint");
//         yawStabSetpoint = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x62, (long) (yawStabSetpoint*1000000));
//         break;
//       case 0x65: // PitchRateStabSetpoint  
//         Serial.println("Pitch rate StabSetpoint");
//         pitchRateSetpoint = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x65, (long) (pitchRateSetpoint*1000000));
//         break;
//       case 0x67: // YawRateSetpoint  
//         Serial.println("YawRateSetpoint");
//         yawRateSetpoint = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x67, (long) (yawRateSetpoint*1000000));
//         break;
//       case 0x63: // AltStabSetpoint  
//         Serial.println("Alt StabSetpoint");
//         altStabSetpoint = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000; 
//                       //= (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x63, (long) (altStabSetpoint*1000000));
//         break;  
//       case 0x64: // VelStabSetpoint  
//         Serial.println("Vel StabSetpoint");
//         velStabSetpoint = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x64, (long) (velStabSetpoint*1000000));
//         break;  

//       case 0x79: // Roll Stab_P ----------------------------- ROLL  ---------------------------------------------
//         Serial.println("Roll Stab_P");
//         rollStabP = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x79, (long) (rollStabP*1000000));
//         break;
//       case 0x80: // Roll Stab_I 
//         Serial.println("Roll Stab_I");
//         rollStabI = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x80, (long) (rollStabI*1000000));
//         break;
//       case 0x81: // Roll Stab_D 
//         Serial.println("Roll Stab_D");
//         rollStabD = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x81, (long) (rollStabD*1000000));
//         break;
//       case 0x82: // Roll Rate_P 
//         Serial.println("Roll Rate_P");
//         rollRateP = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x82, (long) (rollRateP*1000000));
//         break;
//       case 0x83: // Roll Rate_I 
//         Serial.println("Roll Rate_I");
//         rollRateI = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x83, (long) (rollRateI*1000000));
//         break;
//       case 0x84: // Roll Rate_D 
//         Serial.println("Roll Rate_D");
//         rollRateD = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x84, (long) (rollRateD*1000000));
//         break;

//       case 0x73: // Pitch Stab_P ----------------------------- PITCH ---------------------------------------------
//         Serial.println("Pitch Stab_P");
//         pitchStabP = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000*1000; //added x1000 scaling here
//         sendAcknowledgement(0x100, 0x73, (long) (pitchStabP*1000000));
//         break;
//       case 0x74: // Pitch Stab_I 
//         Serial.println("Pitch Stab_I");
//         pitchStabI = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x74, (long) (pitchStabI*1000000));
//         break;
//       case 0x75: // Pitch Stab_D 
//         Serial.println("Pitch Stab_D");
//         pitchStabD = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x75, (long) (pitchStabD*1000000));
//         break;
//       case 0x76: // Pitch Rate_P 
//         Serial.println("Pitch Rate_P");
//         pitchRateP = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x76, (long) (pitchRateP*1000000));
//         break;
//       case 0x77: // Pitch Rate_I 
//         Serial.println("Pitch Rate_I");
//         pitchRateI = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x77, (long) (pitchRateI*1000000));
//         break;
//       case 0x78: // Pitch Rate_D 
//         Serial.println("Pitch Rate_D");
//         pitchRateD = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x78, (long) (pitchRateD*1000000));
//         break;
        
//       case 0x85: // Yaw Stab_P ----------------------------- YAW   ---------------------------------------------
//         Serial.println("Yaw Stab_P");
//         yawStabP = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x85, (long) (yawStabP*1000000));
//         break;
//       case 0x86: // Yaw Stab_I 
//         Serial.println("Yaw Stab_I");
//         yawStabI = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x86, (long) (yawStabI*1000000));
//         break;
//       case 0x87: // Yaw Stab_D 
//         Serial.println("Yaw Stab_D");
//         yawStabD = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x87, (long) (yawStabD*1000000));
//         break;
//       case 0x88: // yaw Rate_P 
//         Serial.println("Yaw Rate_P");
//         yawRateP = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x88, (long) (yawRateP*1000000));
//         break;
//       case 0x89: // Yaw Rate_I 
//         Serial.println("Yaw Rate_I");
//         yawRateI = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x89, (long) (yawRateI*1000000));
//         break;
//       case 0x90: // Yaw Rate_D 
//         Serial.println("Yaw Rate_D");
//         yawRateD = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x90, (long) (yawRateD*1000000));
//         break;
        
//         case 0x94: // Alt Stab_P ----------------------------- ALTITUDE ---------------------------------------------
//         Serial.println("Alt Stab_P");
//         altStabP = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x94, (long) (altStabP*1000000));
//         break;
//       case 0x95: // Alt Stab_I 
//         Serial.println("Alt Stab_I");
//         altStabI = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x95, (long) (altStabI*1000000));
//         break;
//       case 0x96: // Alt Stab_D 
//         Serial.println("Alt Stab_D");
//         altStabD = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x96, (long) (altStabD*1000000));
//         break;
        
//       case 0x91: // Vel Stab_P ----------------------------- VELOCITY ---------------------------------------------
//         Serial.println("Vel Stab_P");
//         velStabP = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x91, (long) (velStabP*1000000));
//         break;
//       case 0x92: // Vel Stab_I 
//         Serial.println("Vel Stab_I");
//         velStabI = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x92, (long) (velStabI*1000000));
//         break;
//       case 0x93: // Vel Stab_D 
//         Serial.println("Vel Stab_D");
//         velStabD = (float) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x93, (long) (velStabD*1000000));
//         break;

//       case 0x144: // Rmt roll impact ----------------------------- Remote impact-----------------------------------------
//         Serial.println("Rmt roll impact");
//         rmtRollMax = (int) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x144, (long) (rmtRollMax*1000000));
//         break;
//       case 0x145: // Rmt pitch impact
//         Serial.println("Rmt pitch impact");
//         rmtPitchMax = (int) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x145, (long) (rmtPitchMax*1000000));
//         break;
//       case 0x146: // Rmt yaw impact
//         Serial.println("Rmt yaw impact");
//         rmtYawMax = (int) ((inMsg.buf[0]<<24) | (inMsg.buf[1]<<16) | (inMsg.buf[2]<<8) | (inMsg.buf[3]))/1000000;
//         sendAcknowledgement(0x100, 0x146, (long) (rmtYawMax*1000000));
//         break;

//       default:
//         //Serial.print("Unknown CAN id:"); 
//         //Serial.print(inMsg.id);
//         //Serial.print(" Data:");
//         //hexDump(8, inMsg.buf);
//         break;
//     }
    
//   }

//   //----------------------------FUNDAMENTAL LOOP-----------------------
  
//   elapsedTime = micros() - lastTime;
//   if ((elapsedTime) >= (loopPeriod * 1000)) 
//   {
//      lastTime = micros();

//      //TODO: Add a GPSSoG timeout warning by sending an ack
     
//      msg.buf[0] = highByte(elapsedTime);
//      msg.buf[1] = lowByte(elapsedTime);
//      Can1.write(msg);

//       // controller inputs/feedback
//       rollStabInput   = euler_X;            // 
//       pitchStabInput  = euler_Y;
//       yawStabInput    = euler_Z;
//       rollRateInput   = rot_X;
//       pitchRateInput  = rot_Y;
//       yawRateInput    = rot_Z;
//       velStabInput    = gpsSoG;               //Last read GPS speed in m/s
//       altStabCalc     = ((senixAft/10 - AltAftOffset) + (senixFore/10 - AltForeOffset))/2*cos(pitchStabInput)*cos(rollStabInput); //Scale from Senix oupt
//       altStabInput = (double) altStabCalc; // Use moving average as altStabInput [mm]

      
//      sendPIDVals(); // Send PID output values through CAN
  
//   } // lastTime loop
//   //-------------------------------------------------------------------
  
  
  
// }
