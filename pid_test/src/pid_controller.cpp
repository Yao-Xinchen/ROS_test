#include "rclcpp/rclcpp.hpp"
#include "pid_test/can_driver.hpp"

#include "pid_test/motor_data.hpp"
#include "pid_test/motor_driver.hpp"

#include <can_interface/msg/motor_present.hpp>
#include <rclcpp/timer.hpp>
#include "test_interface/msg/goal_vel.hpp"

class PidController : public rclcpp::Node
{
public:
    PidController(Params params) : Node("pid_controller")
    {
        motor_driver_ = new MotorDriver(2, params);
        frame_init();
        control_timer_ = this->create_wall_timer(std::chrono::milliseconds(CONTROL_R), std::bind(&PidController::control_timer_callback, this));
        feedback_timer_ = this->create_wall_timer(std::chrono::milliseconds(FEEDBACK_R), std::bind(&PidController::feedback_timer_callback, this));
        goal_sub_ = this->create_subscription<test_interface::msg::GoalVel>("goal_vel", 10, std::bind(&PidController::goal_sub_callback, this, std::placeholders::_1));
    }

    ~PidController()
    {
        delete motor_driver_;
    }

private:
    rclcpp::TimerBase::SharedPtr control_timer_; // send control frame regularly
    rclcpp::TimerBase::SharedPtr feedback_timer_; // receive feedback frame regularly
    rclcpp::Subscription<test_interface::msg::GoalVel>::SharedPtr goal_sub_; // receive goal velocity
    MotorDriver* motor_driver_;

    void goal_sub_callback(const test_interface::msg::GoalVel::SharedPtr msg)
    {
        motor_driver_->set_goal(msg->vel);
    }

    void control_timer_callback()
    {
        motor_driver_->write_frame(MotorDriver::tx_frame);
        for (int i = 0; i < 5; i++) {
            MotorDriver::send_frame(MotorDriver::tx_frame);
        }
    }

    void feedback_timer_callback()
    {
        MotorDriver::can_0->get_frame(MotorDriver::rx_frame);
        auto present_data = motor_driver_->process_rx();
        motor_driver_->update_vel(present_data.velocity);
    }

    void frame_init()
    {
        MotorDriver::tx_frame.can_id = 0x200;
        MotorDriver::tx_frame.can_dlc = 8;
        for (int i = 0; i < 8; i++) MotorDriver::tx_frame.data[i] = 0x00;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    Params params;

    if (argc != 5) {
        printf("Usage: %s goal v2c_kp v2c_ki v2c_kd\n", argv[0]);
        return 1;
    }

    params.goal = std::stof(argv[1]);
    params.v2c_kp = std::stof(argv[2]);
    params.v2c_ki = std::stof(argv[3]);
    params.v2c_kd = std::stof(argv[4]);

    auto node = std::make_shared<PidController>(params);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}