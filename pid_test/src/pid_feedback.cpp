#include "rclcpp/rclcpp.hpp"
#include "pid_test/can_driver.hpp"
#include "pid_test/motor_data.hpp"
#include "pid_test/motor_driver.hpp"

#include "can_interface/srv/motor_present.hpp"

#define DT 10

class PidFeedback : public rclcpp::Node
{
public:
    PidFeedback() : Node("pid_feedback")
    {
        srv_ = this->create_service<can_interface::srv::MotorPresent>(
            "motor_present", [this](const can_interface::srv::MotorPresent::Request::SharedPtr request,
                                    can_interface::srv::MotorPresent::Response::SharedPtr response){
                this->srv_callback(response);
            });
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&PidFeedback::timer_callback, this));
    }

private:
    rclcpp::Service<can_interface::srv::MotorPresent>::SharedPtr srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    MotorData present_data;
    MotorDriver* motor_driver_;

    void srv_callback(can_interface::srv::MotorPresent::Response::SharedPtr response)
    {
        response->present_pos = present_data.position;
        response->present_vel = present_data.velocity;
        response->present_tor = present_data.torque;
    }

    void timer_callback()
    {
        MotorDriver::can_0->get_frame(MotorDriver::rx_frame);
        present_data = motor_driver_->process_rx();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pid_feedback");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}