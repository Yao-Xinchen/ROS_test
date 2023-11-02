#include "pid_test/motor_driver.hpp"
#include "pid_test/motor_data.hpp"
#include <linux/can.h>

can_frame MotorDriver::rx_frame;
CanDriver* MotorDriver::can_0 = new CanDriver(0);

MotorDriver::MotorDriver()
{
    id = 2;

    v2c_kp = 0.1;
    v2c_ki = 0.1;
    v2c_kd = 0.1;

    proportional = 0;
    integral = 0;
    derivative = 0;

    present_pos = 0;
    present_vel = 0;
    goal_pos = 0;
    goal_vel = 0;
    current = 0;
    vel_error = 0;
    pos_error = 0;
}

void MotorDriver::set_goal(float vel)
{
    goal_vel = vel;
}

void MotorDriver::update_vel(float vel)
{
    present_vel = vel;
}

float MotorDriver::vel2current(float goal_vel)
{
    float last_vel_error = vel_error;
    vel_error = goal_vel - present_vel;

    proportional = v2c_kp * vel_error;
    integral += v2c_ki * vel_error * DT;
    derivative = v2c_kd * (vel_error - last_vel_error) / DT;

    current = proportional + integral + derivative;

    return current;
}

void MotorDriver::write_frame(can_frame &tx_frame)
{
    current = vel2current(goal_vel);
    std::uint16_t current_data = float_to_uint(current, -TOR_MAX, TOR_MAX, 16);

    tx_frame.data[2*id - 2] = current_data >> 8;
    tx_frame.data[2*id - 1] = current_data & 0xff;
}

void MotorDriver::send_frame(can_frame &tx_frame)
{
    can_0->send_frame(tx_frame);
}

MotorData MotorDriver::process_rx()
{
    MotorData present_data;

    float pos_raw = rx_frame.data[0]<<8 | rx_frame.data[1];
    float vel_raw = rx_frame.data[2]<<8 | rx_frame.data[3];
    float tor_raw = rx_frame.data[4]<<8 | rx_frame.data[5];

    present_data.position = raw2actual(pos_raw, POS_MAX, 16);
    present_data.velocity = raw2actual(vel_raw, VEL_MAX, 16);
    present_data.torque = raw2actual(tor_raw, TOR_MAX, 16);

    return present_data;
}

float MotorDriver::raw2actual(uint16_t raw, float actual_max, uint8_t bits)
{
    return ((float)(raw - (2 << (bits - 2))) * 2 * actual_max)/(float)(2 << (bits - 1));
}

float MotorDriver::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// Converts an unsigned int to a float, given range and number of bits
    float span = x_max - x_min;
    float offset = x_min;
    return (float) x_int * span / (float) ((1<<bits)-1) + offset;
}

int MotorDriver::float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}