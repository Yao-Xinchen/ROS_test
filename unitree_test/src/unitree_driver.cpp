#include "unitree_test/unitree_driver.hpp"

UnitreeDriver::UnitreeDriver() : serial_port("/dev/ttyUSB0")
{
    goal_cmd.motorType = MotorType::GO_M8010_6;
    feedback_data.motorType = MotorType::GO_M8010_6;
    goal_cmd.mode = queryMotorMode(goal_cmd.motorType, MotorMode::FOC);
    goal_cmd.id  = 0;
    goal_cmd.kp  = 0.02;
    goal_cmd.kd  = 0.0;
    goal_cmd.q   = 0.0 * queryGearRatio(MotorType::GO_M8010_6);
    goal_cmd.dq  = 0.0 * queryGearRatio(MotorType::GO_M8010_6);
    goal_cmd.tau = 0.0;
    serial_port.sendRecv(&goal_cmd, &feedback_data);
}

UnitreeDriver::~UnitreeDriver()
{
    goal_cmd.kp  = 0.0;
    goal_cmd.kd  = 0.0;
    goal_cmd.q   = 0.0;
    goal_cmd.dq  = 0.0;
    goal_cmd.tau = 0.0;
    serial_port.sendRecv(&goal_cmd, &feedback_data);
}

void UnitreeDriver::set_goal(float pos, float vel)
{
    goal_cmd.tau = 0.0; // cmd.T
    if (pos != 0.0)
    {
        goal_cmd.kd = 0.0;
        goal_cmd.kp = 0.02;
        goal_cmd.dq = 0.0;
        goal_cmd.q = pos * queryGearRatio(MotorType::GO_M8010_6); // cmd.Pos
    }
    if (vel != 0.0)
    {
        goal_cmd.kd = 0.02;
        goal_cmd.kp = 0.0;
        goal_cmd.q = 0.0;
        goal_cmd.dq = vel * queryGearRatio(MotorType::GO_M8010_6); // cmd.W
    }
    serial_port.sendRecv(&goal_cmd, &feedback_data);
}

void UnitreeDriver::send_recv()
{
    serial_port.sendRecv(&goal_cmd, &feedback_data);
}