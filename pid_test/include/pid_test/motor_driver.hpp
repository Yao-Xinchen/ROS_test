#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "cstdint"
#include "can_driver.hpp"
#include <linux/can.h>
#include <chrono>
#include <thread>
#include <memory>

#include "motor_data.hpp"

#define ENCODER_ANGLE_RATIO 360.0f / 8192.0f
#define REDUCE_RATIO 36.0f

#define CONTROL_R 1 // ms
#define FEEDBACK_R 1 // ms

#define V_MAX 1000
#define I_MAX 20

struct Params
{
    float goal_vel;
    float goal_pos;
    float v2c_kp;
    float v2c_ki;
    float v2c_kd;
};

class MotorDriver
{
public:
    static can_frame tx_frame;
    static can_frame rx_frame;
    static std::unique_ptr<CanDriver> can_0;

    MotorDriver(int id, Params params);
    
    void process_rx();

    void set_goal(float vel, float pos);

    void write_frame();
    static void send_frame();

    MotorData present_data;
private:
    int id;

    float v2c_kp, v2c_ki, v2c_kd;
    float proportional, integral, derivative;
    
    float goal_vel;
    float goal_pos;

    float current;
    float vel_error;
    float pos_error;

    void vel2current();
    void pos2velocity();
};

#endif // MOTOR_DRIVER_HPP