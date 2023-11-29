#ifndef DM_DRIVER_H
#define DM_DRIVER_H

#include "can_driver.hpp"

#include "cstdint"
#include "can_driver.hpp"
#include <bits/stdint-uintn.h>
#include <linux/can.h>
#include <chrono>
#include <thread>

#define START_CMD {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc}
#define STOP_CMD {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd}
#define SAVE_ZERO_CMD {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe}
#define CLEAR_ERROR_CMD {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}

// can be read from the debugging assistant
#define P_MAX 3.141593
#define V_MAX 30.0
#define T_MAX 10.0

#define KP_MIN 0.0
#define KP_MAX 500.0
#define KD_MIN 0.0
#define KD_MAX 5.0

class DmDriver
{
public:
    static can_frame tx_frame;
    int motor_id;
    virtual ~DmDriver();
    void turn_on();
    void turn_off();

    static CanDriver* can_0;

    virtual void set_mode() = 0;
    virtual void set_velocity(float goal_vel) = 0;
    virtual void set_position(float goal_pos) = 0;

    // data conversion provided by DM
    static float uint_to_float(int x_int, float x_min, float x_max, int bits);
    static int float_to_uint(float x, float x_min, float x_max, int bits);
};

class DmMitDriver : public DmDriver
{
private:
    float kp, kd;

public:
    DmMitDriver(int id, float kp, float kd);
    void set_mode() override;
    void set_param_mit(float kp, float kd);
    void set_velocity(float goal_vel) override;
    void set_position(float goal_pos) override;
};

class DmVelDriver : public DmDriver
{
public:
    DmVelDriver(int id);
    void set_mode() override;
    void set_velocity(float goal_vel) override;
    void set_position(float goal_pos) override;
};

#endif // DM_DRIVER_H