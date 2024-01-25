#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "remote_test/uart_driver.hpp"

#define RATE 10 // ms

class UartTest : public rclcpp::Node
{
public:
    UartTest() : Node("uart_test")
    {
        RCLCPP_INFO(this->get_logger(), "uart_test node created");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(RATE),
            std::bind(&UartTest::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<UartDriver> uart_driver_;

    void timer_callback()
    {
        std::string data;
        uart_driver_->read(data);
        if (!data.empty())
        {
            RCLCPP_INFO(this->get_logger(), "uart_test node received: %s", data.c_str());
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}