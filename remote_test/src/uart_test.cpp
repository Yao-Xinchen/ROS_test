#include "rclcpp/rclcpp.hpp"
#include <bits/stdint-uintn.h>
#include <cstdio>
#include <memory>
#include <rclcpp/logging.hpp>
#include <vector>
#include "remote_test/uart_driver.hpp"

#define RATE 200 // ms

class UartTest : public rclcpp::Node
{
public:
    UartTest(std::string name) : Node("uart_test")
    {
        RCLCPP_INFO(this->get_logger(), "uart_test node created");
        uart_driver_ = std::make_unique<UartDriver>(name);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(RATE),
            std::bind(&UartTest::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<UartDriver> uart_driver_;

    void timer_callback()
    {
        // std::vector<uint8_t> data;
        // uart_driver_->read(data);
        // if (!data.empty())
        // {
        //     printf("data received: ");
        //     for (auto &d : data)
        //     {
        //         printf("%d ", d);
        //     }
        //     printf("\n");
        // }

        std::string str = "hello\n";
        std::vector<uint8_t> data(str.begin(), str.end());
        uart_driver_->send(data);
        RCLCPP_INFO(this->get_logger(), "str sent");
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto name = argv[1];
    auto node = std::make_shared<UartTest>(name);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}