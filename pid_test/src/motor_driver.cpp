#include "pid_test/motor_driver.hpp"
#include "pid_test/motor_data.hpp"
#include <cstdint>
#include <cstdio>
#include <linux/can.h>

can_frame MotorDriver::tx_frame;
can_frame MotorDriver::rx_frame;
CanDriver* MotorDriver::can_0 = new CanDriver(0);

#define GOAL_VEL 50

MotorDriver::MotorDriver(int id, Params params)
{
    this->id = id;

    v2c_kp = params.v2c_kp;
    v2c_ki = params.v2c_ki;
    v2c_kd = params.v2c_kd;

    proportional = 0;
    integral = 0;

    present_vel = 0;
    goal_vel = params.goal;
    current = 0;
    vel_error = 0;
}

void MotorDriver::set_goal(float vel)
{
    goal_vel = vel;
}

void MotorDriver::update_vel(float vel)
{
    present_vel = vel;
}

float MotorDriver::vel2current(const float goal_vel)
{
    float previous_vel_error = vel_error;
    vel_error = goal_vel - present_vel;

    proportional = v2c_kp * vel_error;
    integral += v2c_ki * vel_error * CONTROL_R;
    derivative = v2c_kd * (vel_error - previous_vel_error) / CONTROL_R;

    current = proportional + integral + derivative;
    // current = proportional;

    if (current > 20) current = 20;
    else if (current < -20) current = -20;

    return current;
}

void MotorDriver::write_frame(can_frame &tx_frame)
{
    current = vel2current(goal_vel);
    printf("proportional: %f, present_vel: %f, goal_vel: %f, current: %f\n", proportional, present_vel, goal_vel, current);

    int current_data = current / 20 * 16384; // int16_t !!! not uint16_t

    tx_frame.data[2*id - 2] = (uint8_t)(current_data >> 8);
    tx_frame.data[2*id - 1] = (uint8_t)(current_data & 0xff);
}

void MotorDriver::send_frame(const can_frame &tx_frame)
{
    can_0->send_frame(tx_frame);
}

MotorData MotorDriver::process_rx()
{
    MotorData present_data;

    int16_t pos_raw = rx_frame.data[0]<<8 | rx_frame.data[1];
    int16_t vel_raw = rx_frame.data[2]<<8 | rx_frame.data[3];
    int16_t tor_raw = rx_frame.data[4]<<8 | rx_frame.data[5];

    present_data.position = (float)pos_raw * ENCODER_ANGLE_RATIO;
    present_data.velocity = (float)vel_raw * 3.1415926f / 30.0f; // rpm to rad/s, 2*pi/60
    present_data.torque = (float)tor_raw * 16384 / 20; // actually current, Ampere

    return present_data;
}