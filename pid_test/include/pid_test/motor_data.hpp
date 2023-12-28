#ifndef MOTOR_DATA_HPP
#define MOTOR_DATA_HPP

#include <cmath>

#define ITV 20

class MotorData
{
public:
    float torque;
    float velocity;
    float position;

    MotorData()
    {
        torque = 0;
        velocity = 0;
        position = 0;
    }

    void update_pos(float pos)
    {
        // pos is from (0, 360)
        // calculate the cumulative angle
        int round = std::floor(position / 360);
        
        if (std::fmod(position, 360) > 360 - ITV && pos < ITV)
        {
            position = 360 * (round + 1) + pos;
        } else if (std::fmod(position, 360) < ITV && pos > 360 - ITV)
        {
            position = 360 * (round - 1) + pos;
        } else
        {
            position = 360 * round + pos;
        }
    }
};

#endif