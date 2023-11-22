#include "rclcpp/rclcpp.hpp"
#include "pid_test/can_driver.hpp"

#include "pid_test/motor_data.hpp"
#include "pid_test/motor_driver.hpp"

#include "can_interface/srv/motor_present.hpp"
#include "test_interface/msg/goal_vel.hpp"
#include <cstdlib>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/logging.hpp>

class PidController : public rclcpp::Node
{
public:
    PidController() : Node("pid_controller")
    {
        float v2c_params[3] = {0.1, 0.1, 0.1};
        motor_driver_ = new MotorDriver(2, v2c_params);
        frame_init();
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cli_ = this->create_client<can_interface::srv::MotorPresent>("motor_present");
        wait();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&PidController::timer_callback, this));
        sub_ = this->create_subscription<test_interface::msg::GoalVel>("goal_vel", 10, std::bind(&PidController::sub_callback, this, std::placeholders::_1));
    }

    ~PidController()
    {
        delete motor_driver_;
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<can_interface::srv::MotorPresent>::SharedPtr cli_;
    rclcpp::Subscription<test_interface::msg::GoalVel>::SharedPtr sub_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    MotorDriver* motor_driver_;

    void sub_callback(const test_interface::msg::GoalVel::SharedPtr msg)
    {
        motor_driver_->set_goal(msg->vel);
    }

    void timer_callback()
    {
        auto request = std::make_shared<can_interface::srv::MotorPresent::Request>();
        auto cli_callback = [&, this](rclcpp::Client<can_interface::srv::MotorPresent>::SharedFuture inner_future)
        {
            auto result = inner_future.get();
            motor_driver_->update_vel(result->present_vel);
            motor_driver_->write_frame(MotorDriver::tx_frame);
            MotorDriver::send_frame(MotorDriver::tx_frame);
            RCLCPP_INFO(this->get_logger(), "Frame sent.");
        };
        auto future_result = cli_->async_send_request(request, cli_callback);
    }

    void frame_init()
    {
        MotorDriver::tx_frame.can_id = 0x200;
        MotorDriver::tx_frame.can_dlc = 8;
        for (int i = 0; i < 8; i++) MotorDriver::tx_frame.data[i] = 0x00;
    }

    void wait()
    {
        while (!cli_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
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