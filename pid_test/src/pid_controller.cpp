#include "rclcpp/rclcpp.hpp"
#include "pid_test/can_driver.hpp"

#include "pid_test/motor_data.hpp"
#include "pid_test/motor_driver.hpp"

#include <can_interface/msg/motor_present.hpp>
#include "test_interface/msg/goal_vel.hpp"

class PidController : public rclcpp::Node
{
public:
    PidController() : Node("pid_controller")
    {
        float v2c_params[2] = {0.1, 0.1};
        motor_driver_ = new MotorDriver(2, v2c_params);
        frame_init();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(CONTROL_R), std::bind(&PidController::timer_callback, this));
        goal_sub_ = this->create_subscription<test_interface::msg::GoalVel>("goal_vel", 10, std::bind(&PidController::goal_sub_callback, this, std::placeholders::_1));
        feedback_sub_ = this->create_subscription<can_interface::msg::MotorPresent>("motor_present", 10, std::bind(&PidController::feedback_sub_callback, this, std::placeholders::_1));
    }

    ~PidController()
    {
        delete motor_driver_;
    }

private:
    rclcpp::TimerBase::SharedPtr timer_; // send control frame regularly
    rclcpp::Subscription<test_interface::msg::GoalVel>::SharedPtr goal_sub_; // receive goal velocity
    rclcpp::Subscription<can_interface::msg::MotorPresent>::SharedPtr feedback_sub_; // update local data
    MotorDriver* motor_driver_;

    void goal_sub_callback(const test_interface::msg::GoalVel::SharedPtr msg)
    {
        motor_driver_->set_goal(msg->vel);
    }

    void feedback_sub_callback(const can_interface::msg::MotorPresent::SharedPtr msg)
    {
        motor_driver_->update_vel(msg->present_vel);
    }

    void timer_callback()
    {

        motor_driver_->write_frame(MotorDriver::tx_frame);
        MotorDriver::send_frame(MotorDriver::tx_frame);
        // RCLCPP_INFO(this->get_logger(), "Present_vel: %f", motor_driver_->present_vel);
        // RCLCPP_INFO(this->get_logger(), "Current_data: %d", data);
        // RCLCPP_INFO(this->get_logger(), "Current: %f", current);
        // RCLCPP_INFO(this->get_logger(), "tx_frame[2] = %d, tx_frame[3] = %d", MotorDriver::tx_frame.data[2], MotorDriver::tx_frame.data[3]);
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
    auto node = std::make_shared<PidController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}