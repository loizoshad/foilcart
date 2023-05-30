#include <FlexCAN_T4.h>
#include <elapsedMillis.h>
#include <stdint.h>
#include <vector>

// CAN Message schema, see Dropbox
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

static CAN_message_t msg120;    // Senix altitude fore 
static CAN_message_t msg121;    // Senix altitude aft

static CAN_message_t msg200;    // EKF states 1-2
static CAN_message_t msg201;    // EKF states 3-4
static CAN_message_t msg202;    // EKF states 5-6
static CAN_message_t msg203;    // EKF states 7-8
static CAN_message_t msg204;    // EKF states 9-10
static CAN_message_t msg205;    // EKF states 11-12

static CAN_message_t msg105;    // Control inputs 1-4
static CAN_message_t msg106;    // Control inputs 5-8


void init_can() 
{

    Can1.begin();
    Can1.setBaudRate(250000);

    // EKF CAN-messages:
    // EKF 1-2
    msg200.flags.extended = 0;
    msg200.id = 0x200;
    msg200.len = 8;
    msg200.buf[0] = 0;
    msg200.buf[1] = 0;
    msg200.buf[2] = 0;
    msg200.buf[3] = 0;
    msg200.buf[4] = 0;
    msg200.buf[5] = 0;
    msg200.buf[6] = 0;
    msg200.buf[7] = 0;

    // EKF 3-4
    msg201.flags.extended = 0;
    msg201.id = 0x201;
    msg201.len = 8;
    msg201.buf[0] = 0;
    msg201.buf[1] = 0;
    msg201.buf[2] = 0;
    msg201.buf[3] = 0;
    msg201.buf[4] = 0;
    msg201.buf[5] = 0;
    msg201.buf[6] = 0;
    msg201.buf[7] = 0;

    //EKF 5-6
    msg202.flags.extended = 0;
    msg202.id = 0x202;
    msg202.len = 8;
    msg202.buf[0] = 0;
    msg202.buf[1] = 0;
    msg202.buf[2] = 0;
    msg202.buf[3] = 0;
    msg202.buf[4] = 0;
    msg202.buf[5] = 0;
    msg202.buf[6] = 0;
    msg202.buf[7] = 0;

    //EKF 7-8
    msg203.flags.extended = 0;
    msg203.id = 0x203;
    msg203.len = 8;
    msg203.buf[0] = 0;
    msg203.buf[1] = 0;
    msg203.buf[2] = 0;
    msg203.buf[3] = 0;
    msg203.buf[4] = 0;
    msg203.buf[5] = 0;
    msg203.buf[6] = 0;
    msg203.buf[7] = 0;

    //EKF 9-10
    msg204.flags.extended = 0;
    msg204.id = 0x204;
    msg204.len = 8;
    msg204.buf[0] = 0;
    msg204.buf[1] = 0;
    msg204.buf[2] = 0;
    msg204.buf[3] = 0;
    msg204.buf[4] = 0;
    msg204.buf[5] = 0;
    msg204.buf[6] = 0;
    msg204.buf[7] = 0;

    //EKF 11-12
    msg205.flags.extended = 0;
    msg205.id = 0x205;
    msg205.len = 8;
    msg205.buf[0] = 0;
    msg205.buf[1] = 0;
    msg205.buf[2] = 0;
    msg205.buf[3] = 0;
    msg205.buf[4] = 0;
    msg205.buf[5] = 0;
    msg205.buf[6] = 0;
    msg205.buf[7] = 0;



}


void write_can(std::vector<float> state_CAN)
{

    // Extract the states and convert each into a f

    // Send x and vx at msg200
    msg200.id = 0x200;
    msg200.len = 8;

  // Shift DEC comma to the right before converting to int
    long x          = (int32_t)( 1000000.0f * state_CAN[0]  ) ;
    long y          = (int32_t)( 1000000.0f * state_CAN[1]  ) ;
    long z          = (int32_t)( 1000000.0f * state_CAN[2]  ) ;
    long roll       = (int32_t)( 1000000.0f * state_CAN[3]  ) ;
    long pitch      = (int32_t)( 1000000.0f * state_CAN[4]  ) ;
    long yaw        = (int32_t)( 1000000.0f * state_CAN[5]  ) ;
    long vx         = (int32_t)( 1000000.0f * state_CAN[6]  ) ;
    long vy         = (int32_t)( 1000000.0f * state_CAN[7]  ) ;
    long vz         = (int32_t)( 1000000.0f * state_CAN[8]  ) ;
    long roll_rate  = (int32_t)( 1000000.0f * state_CAN[9]  ) ;
    long pitch_rate = (int32_t)( 1000000.0f * state_CAN[10] ) ;
    long yaw_rate   = (int32_t)( 1000000.0f * state_CAN[11] ) ;

    // x and vx
    msg200.buf[0] = x >> 24;
    msg200.buf[1] = x >> 16;
    msg200.buf[2] = x >> 8;
    msg200.buf[3] = x >> 0;
    msg200.buf[4] = vx >> 24;
    msg200.buf[5] = vx >> 16;
    msg200.buf[6] = vx >> 8;
    msg200.buf[7] = vx >> 0;
    
    // y and vy
    msg201.buf[0] = y >> 24;
    msg201.buf[1] = y >> 16;
    msg201.buf[2] = y >> 8;
    msg201.buf[3] = y >> 0;
    msg201.buf[4] = vy >> 24;
    msg201.buf[5] = vy >> 16;
    msg201.buf[6] = vy >> 8;
    msg201.buf[7] = vy >> 0;

    // z and vz
    msg202.buf[0] = z >> 24;
    msg202.buf[1] = z >> 16;
    msg202.buf[2] = z >> 8;
    msg202.buf[3] = z >> 0;
    msg202.buf[4] = vz >> 24;
    msg202.buf[5] = vz >> 16;
    msg202.buf[6] = vz >> 8;
    msg202.buf[7] = vz >> 0;

    // roll and roll_rate
    msg203.buf[0] = roll >> 24;
    msg203.buf[1] = roll >> 16;
    msg203.buf[2] = roll >> 8;
    msg203.buf[3] = roll >> 0;
    msg203.buf[4] = roll_rate >> 24;
    msg203.buf[5] = roll_rate >> 16;
    msg203.buf[6] = roll_rate >> 8;
    msg203.buf[7] = roll_rate >> 0;

    // pitch and pitch_rate
    msg204.buf[0] = pitch >> 24;
    msg204.buf[1] = pitch >> 16;
    msg204.buf[2] = pitch >> 8;
    msg204.buf[3] = pitch >> 0;
    msg204.buf[4] = pitch_rate >> 24;
    msg204.buf[5] = pitch_rate >> 16;
    msg204.buf[6] = pitch_rate >> 8;
    msg204.buf[7] = pitch_rate >> 0;

    // yaw and yaw_rate
    msg205.buf[0] = yaw >> 24;
    msg205.buf[1] = yaw >> 16;
    msg205.buf[2] = yaw >> 8;
    msg205.buf[3] = yaw >> 0;
    msg205.buf[4] = yaw_rate >> 24;
    msg205.buf[5] = yaw_rate >> 16;
    msg205.buf[6] = yaw_rate >> 8;
    msg205.buf[7] = yaw_rate >> 0;

    // Send the messages
    Can1.write(msg200);
    Can1.write(msg201);
    Can1.write(msg202);
    Can1.write(msg203);
    Can1.write(msg204);
    Can1.write(msg205);

}

    
void read_can( float* fore_alt, float* aft_alt, float* wing_angle, float* rudder_angle, float* elevator_angle, float* throttle)
{                  
    CAN_message_t inMsg;
    while ( Can1.read(inMsg)!=0 ) {
        
        switch(inMsg.id) 
        {
        // Senix sensors data:
        case 0x120: // Fore Senix sensor altitude
        {
            // TODO Implement the correct conversion
            int16_t fore_alt_int = (int16_t) (inMsg.buf[0] << 8 | inMsg.buf[1]);
            *fore_alt = (float) fore_alt_int;

            // for (int i = 0; i < inMsg.len; i++)
            // {
            //     Serial.print(inMsg.buf[i], HEX);
            //     Serial.print(" ");
            // }      
            // Serial.print("fore_alt: ");
            // Serial.println(*fore_alt);
            break;
        }
        case 0x121: // Aft Senix sensor altitude
        {
            // TODO: Implement the correct conversion
            int16_t aft_alt_int = (int16_t) (inMsg.buf[0] << 8 | inMsg.buf[1]);
            *aft_alt = (float) aft_alt_int;

            // for (int i = 0; i < inMsg.len; i++)
            // {
            //     Serial.print(inMsg.buf[i], HEX);
            //     Serial.print(" ");
            // }
            // Serial.print("aft_alt: ");
            // Serial.println(*aft_alt);
            break;
        }
        // Control input data:
        case 0x105: // Control input (Roll, Elevator, Throttle, Rudder)
        {
            // TODO:
            /* Mesage Structure
            B7 : Roll High      | B6 : Roll Low | 
            B5 : Elevator High  | B4 : Elevator Low | 
            B3 : Throttle High  | B2 : Throttle Low | 
            B1 : Rudder High    | B0 : Rudder Low |
            */
            break;
        }
        case 0x106: // Control input (Wing angle, Flaps, Control type, Ch8)
        {
            // TODO:
            /* Message Structure
            B7 : Wing angle High      | B6 : Wing angle Low |
            B5 : Flaps High           | B4 : Flaps Low |
            B3 : Control type High    | B2 : Control type Low |
            B1 : Ch8 High             | B0 : Ch8 Low |
            */
            break;
        }
        default:
        {
            // Serial.print("ERROR: inMsd.id = ");
            // Serial.println(inMsg.id, HEX);
            break;            
        }
        }
    }
}


