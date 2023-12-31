#include "rclcpp/rclcpp.hpp"
#include "pid_test/can_driver.hpp"
#include "pid_test/motor_data.hpp"
#include "pid_test/motor_driver.hpp"

#include "can_interface/msg/motor_present.hpp"

class PidFeedback : public rclcpp::Node
{
public:
    PidFeedback() : Node("pid_feedback")
    {
        Params params;
        params.goal_vel = 0;
        params.goal_pos = 0;
        params.v2c_kp = 0;
        params.v2c_ki = 0;
        params.v2c_kd = 0;
        motor_driver_ = new MotorDriver(2, params);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(FEEDBACK_R), std::bind(&PidFeedback::timer_callback, this));
        pub_ = this->create_publisher<can_interface::msg::MotorPresent>("motor_present", 10);
    }

    ~PidFeedback()
    {
        delete motor_driver_;
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<can_interface::msg::MotorPresent>::SharedPtr pub_;

    MotorDriver* motor_driver_;
    can_interface::msg::MotorPresent motor_msg;

    void timer_callback()
    {
        MotorDriver::can_0->get_frame(MotorDriver::rx_frame);
        motor_driver_->process_rx();
        motor_msg.present_pos = motor_driver_->present_data.position;
        motor_msg.present_vel = motor_driver_->present_data.velocity;
        motor_msg.present_tor = motor_driver_->present_data.torque;
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