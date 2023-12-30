#include "pid_test/motor_driver.hpp"
#include "pid_test/motor_data.hpp"
#include <cstdint>
#include <cstdio>
#include <linux/can.h>
#include <stdio.h>

can_frame MotorDriver::tx_frame;
can_frame MotorDriver::rx_frame;
std::unique_ptr<CanDriver> MotorDriver::can_0 = std::make_unique<CanDriver>(0);

#define GOAL_VEL 50

MotorDriver::MotorDriver(int id, Params params)
{
    this->id = id;

    v2c_kp = params.v2c_kp;
    v2c_ki = params.v2c_ki;
    v2c_kd = params.v2c_kd;
    p2v_kp = params.p2v_kp;
    p2v_ki = params.p2v_ki;
    p2v_kd = params.p2v_kd;

    v2c_p = 0;
    v2c_i = 0;
    v2c_d = 0;
    p2v_p = 0;
    p2v_i = 0;
    p2v_d = 0;

    goal_vel = params.goal_vel;
    goal_pos = params.goal_pos;
    current = 0;
    vel_error = 0;
    pos_error = 0;
}

void MotorDriver::set_goal(float vel, float pos)
{
    goal_vel = vel;
    goal_pos = pos;
}

void MotorDriver::vel2current()
{
    float previous_vel_error = vel_error;
    vel_error = goal_vel - present_data.velocity;

    v2c_p = v2c_kp * vel_error;
    v2c_i += v2c_ki * vel_error * CONTROL_R;
    v2c_d = v2c_kd * (vel_error - previous_vel_error) / CONTROL_R;

    this->current = v2c_p + v2c_i + v2c_d;

    if (current > I_MAX) current = I_MAX;
    else if (current < -I_MAX) current = -I_MAX;
}

void MotorDriver::pos2velocity()
{
    float previous_pos_error = pos_error;
    pos_error = goal_pos - present_data.position;

    p2v_p = p2v_kp * pos_error;
    p2v_i += p2v_ki * pos_error * CONTROL_R;
    p2v_d = p2v_kd * (pos_error - previous_pos_error) / CONTROL_R;

    this->goal_vel = p2v_p + p2v_i + p2v_d;

    if (goal_vel > V_MAX) goal_vel = V_MAX;
    else if (goal_vel < -V_MAX) goal_vel = -V_MAX;
}

void MotorDriver::write_frame()
{
    if (goal_pos != 0.0) pos2velocity();
    vel2current();

    printf("present_pos: %f, goal_pos: %f, present_vel: %f, goal_vel: %f, current: %f\n", present_data.position, goal_pos, present_data.velocity, goal_vel, current);

    int16_t current_data = current / I_MAX * 16384; // int16_t !!! not uint16_t

    this->tx_frame.data[2*id - 2] = (uint8_t)(current_data >> 8);
    this->tx_frame.data[2*id - 1] = (uint8_t)(current_data & 0xff);
}

void MotorDriver::send_frame()
{
    can_0->send_frame(tx_frame);
}

void MotorDriver::process_rx()
{
    if ((int)rx_frame.can_id == 0x200 + id)
    {
        int16_t pos_raw = rx_frame.data[0]<<8 | rx_frame.data[1];
        int16_t vel_raw = rx_frame.data[2]<<8 | rx_frame.data[3];
        int16_t tor_raw = rx_frame.data[4]<<8 | rx_frame.data[5];

        present_data.update_pos((float)pos_raw * ENCODER_ANGLE_RATIO);
        present_data.velocity = (float)vel_raw * 3.1415926f / 30.0f; // rpm to rad/s, 2*pi/60
        present_data.torque = (float)tor_raw * 16384 / I_MAX; // actually current, Ampere
    }
}