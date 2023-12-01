#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "cstdint"
#include "can_driver.hpp"
#include <linux/can.h>
#include <chrono>
#include <thread>

#include "motor_data.hpp"

#define ENCODER_ANGLE_RATIO 360.0f / 8192.0f
#define REDUCE_RATIO 36.0f

#define CONTROL_R 1 // ms
#define FEEDBACK_R 1 // ms

struct Params
{
    float goal;
    float v2c_kp;
    float v2c_ki;
    float v2c_kd;
};

class MotorDriver
{
public:
    static can_frame tx_frame;
    static can_frame rx_frame;
    static CanDriver* can_0;

    MotorDriver(int id, Params params);
    
    void process_rx();

    void set_goal(float vel);

    void write_frame(can_frame &tx_frame);
    static void send_frame(const can_frame &tx_frame);

    MotorData present_data;
private:
    int id;

    float v2c_kp, v2c_ki, v2c_kd;
    float proportional, integral, derivative;
    
    float goal_vel;

    float current;
    float vel_error;

    float vel2current(const float goal_vel);
};

#endif // MOTOR_DRIVER_HPP