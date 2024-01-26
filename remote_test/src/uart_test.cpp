#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <rclcpp/logging.hpp>
#include "remote_test/uart_driver.hpp"

#define RATE 200 // ms

class UartTest : public rclcpp::Node
{
public:
    UartTest() : Node("uart_test")
    {
        RCLCPP_INFO(this->get_logger(), "uart_test node created");
        uart_driver_ = std::make_unique<UartDriver>();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(RATE),
            std::bind(&UartTest::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<UartDriver> uart_driver_;

    void timer_callback()
    {
        // std::string data;
        // uart_driver_->read(data);
        // if (!data.empty())
        // {
        //     RCLCPP_INFO(this->get_logger(), "uart_test node received: %s", data.c_str());
        // }

        std::string str = "hello";
        uart_driver_->send(str);
        RCLCPP_INFO(this->get_logger(), "str sent");
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UartTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}