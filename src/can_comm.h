#ifndef CAN_COMM_H
#define CAN_COMM_H

void init_can();
void read_can( float* fore_alt, float* aft_alt, float* wing_angle, float* rudder_angle, float* elevator_angle, float* throttle);
void write_can(std::vector<float> state_CAN);

#endif