#include "rclcpp/rclcpp.hpp"
#include "pid_test/can_driver.hpp"
#include "pid_test/motor_data.hpp"
#include "pid_test/motor_driver.hpp"

#include "can_interface/msg/motor_present.hpp"
#include <can_interface/msg/detail/motor_present__struct.hpp>
#include <rclcpp/publisher.hpp>

#define PUB_RATE 1 // 1ms 

class PidFeedback : public rclcpp::Node
{
public:
    PidFeedback() : Node("pid_feedback")
    {
        float v2c_params[2] = {10, 0.1};
        motor_driver_ = new MotorDriver(2, v2c_params);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(PUB_RATE), std::bind(&PidFeedback::timer_callback, this));
        pub_ = this->create_publisher<can_interface::msg::MotorPresent>("motor_present", 10);
    }

    ~PidFeedback()
    {
        delete motor_driver_;
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<can_interface::msg::MotorPresent>::SharedPtr pub_;

    MotorData present_data;
    MotorDriver* motor_driver_;
    can_interface::msg::MotorPresent motor_msg;

    void timer_callback()
    {
        MotorDriver::can_0->get_frame(MotorDriver::rx_frame);
        present_data = motor_driver_->process_rx();
        motor_msg.present_pos = present_data.position;
        motor_msg.present_vel = present_data.velocity;
        motor_msg.present_tor = present_data.torque;
        pub_->publish(motor_msg);
        // RCLCPP_INFO(this->get_logger(), "Position: %f, Velocity: %f, Torque: %f", present_data.position, present_data.velocity, present_data.torque);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PidFeedback>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}