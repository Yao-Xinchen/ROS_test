#include "rclcpp/rclcpp.hpp"
#include "pid_test/can_driver.hpp"

#include "pid_test/motor_data.hpp"
#include "pid_test/motor_driver.hpp"

#include "can_interface/srv/motor_present.hpp"
#include "test_interface/msg/goal_vel.hpp"

class PidFeedback : public rclcpp::Node
{
public:
    PidFeedback() : Node("pid_feedback")
    {
        float v2c_params[3] = {0.1, 0.1, 0.1};
        motor_driver_ = new MotorDriver(2, v2c_params);
        cli_ = this->create_client<can_interface::srv::MotorPresent>("motor_present");
        sub_ = this->create_subscription<test_interface::msg::GoalVel>("goal_vel", 10, std::bind(&PidFeedback::sub_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&PidFeedback::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<can_interface::srv::MotorPresent>::SharedPtr cli_;
    rclcpp::Subscription<test_interface::msg::GoalVel>::SharedPtr sub_;
    MotorDriver* motor_driver_;

    void sub_callback(const test_interface::msg::GoalVel::SharedPtr msg)
    {
        motor_driver_->set_goal(msg->vel);
    }

    void timer_callback()
    {
        auto request = std::make_shared<can_interface::srv::MotorPresent::Request>();
        auto response = cli_->async_send_request(request);
        auto result = response.get();

        motor_driver_->update_vel(result->present_vel);
        motor_driver_->write_frame(MotorDriver::tx_frame);
        MotorDriver::send_frame(MotorDriver::tx_frame);
    }

    void frame_init()
    {
        MotorDriver::tx_frame.can_id = 0x200;
        MotorDriver::tx_frame.can_dlc = 8;
        for (int i = 0; i < 8; i++) MotorDriver::tx_frame.data[i] = 0x00;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pid_feedback");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}