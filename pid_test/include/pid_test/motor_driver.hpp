#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "cstdint"
#include "can_driver.hpp"
#include <bits/stdint-uintn.h>
#include <linux/can.h>
#include <chrono>
#include <thread>

#include "motor_data.hpp"

#define ENCODER_ANGLE_RATIO 360.0f / 8192.0f
#define REDUCE_RATIO 36.0f
#define DT 1

class MotorDriver
{
public:
    static can_frame tx_frame;
    static can_frame rx_frame;
    static CanDriver* can_0;

    MotorDriver(int id, float v2c[2]);
    
    MotorData process_rx();

    void set_goal(float vel);
    void update_vel(float vel);

    float write_frame(can_frame &tx_frame);
    static void send_frame(can_frame &tx_frame);
    
private:
    int id;

    float v2c_kp, v2c_ki;
    float proportional, integral;

    float present_pos;
    float present_vel;
    float goal_pos;
    float goal_vel;
    float current;
    float vel_error;

    float vel2current(float goal_vel);
};

#endif // MOTOR_DRIVER_HPP