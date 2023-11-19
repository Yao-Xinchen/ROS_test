#include "rclcpp/rclcpp.hpp"
#include "test_interface/srv/basic_srv.hpp"
#include <rclcpp/client.hpp>
#include <rclcpp/timer.hpp>

class BasicClient : public rclcpp::Node
{
public:
    BasicClient() : Node("basic_client")
    {
        RCLCPP_INFO(this->get_logger(), "Hello from %s", this->get_name());
        cli_ = this->create_client<test_interface::srv::BasicSrv>("basic_srv");
        wait();
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&BasicClient::timer_callback, this));
    }

private:
    rclcpp::Client<test_interface::srv::BasicSrv>::SharedPtr cli_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback()
    {
        auto request = std::make_shared<test_interface::srv::BasicSrv::Request>();
        auto result = cli_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Request sent.");
        // auto response = result.get();
        // RCLCPP_INFO(this->get_logger(), "Response received: %d", response->a);
        if (result.valid())
        {
            RCLCPP_INFO(this->get_logger(), "Result is valid.");
            RCLCPP_INFO(this->get_logger(), "Response received: %d", result.get()->a);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service basic_srv");
        }
    }

    void wait()
    {
        while (!cli_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                // // exit the main
                // exit(EXIT_SUCCESS);
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BasicClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}