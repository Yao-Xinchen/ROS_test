#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "cstdint"
#include "can_driver.hpp"
#include <bits/stdint-uintn.h>
#include <linux/can.h>
#include <chrono>
#include <thread>

#include "motor_data.hpp"

#define POS_MAX 360 // degree
#define VEL_MAX 10000 // rpm, not sure
#define TOR_MAX 20 // actually current, A

#define DT 10

class MotorDriver
{
public:
    static can_frame tx_frame;
    static can_frame rx_frame;
    static CanDriver* can_0;

    MotorDriver();
    
    MotorData process_rx();

    float raw2actual(uint16_t raw, float actual_max, uint8_t bits);

    void set_goal(float vel);
    void update_vel(float vel);

    void write_frame(can_frame &tx_frame);
    static void send_frame(can_frame &tx_frame);

    static float uint_to_float(int x_int, float x_min, float x_max, int bits);
    static int float_to_uint(float x, float x_min, float x_max, int bits);
    
private:
    int id;

    float v2c_kp, v2c_ki, v2c_kd;
    float proportional, integral, derivative;

    float present_pos;
    float present_vel;
    float goal_pos;
    float goal_vel;
    float current;
    float vel_error;
    float pos_error;

    float vel2current(float goal_vel);
};

#endif // MOTOR_DRIVER_HPP