#include "rclcpp/rclcpp.hpp"
#include "test_interface/srv/basic_srv.hpp"
#include <future>

class BasicClient : public rclcpp::Node
{
public:
    BasicClient() : Node("basic_client")
    {
        RCLCPP_INFO(this->get_logger(), "Hello from %s", this->get_name());
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cli_ = this->create_client<test_interface::srv::BasicSrv>("basic_srv", rmw_qos_profile_services_default, cb_group_);
        inner_request = std::make_shared<test_interface::srv::BasicSrv::Request>();
        wait();
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&BasicClient::timer_callback, this));
    }

private:
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::Client<test_interface::srv::BasicSrv>::SharedPtr cli_;
    rclcpp::TimerBase::SharedPtr timer_;
    test_interface::srv::BasicSrv::Request::SharedPtr inner_request;

    void timer_callback()
    {
        int b = 0;
        auto inner_client_callback = [&,this](rclcpp::Client<test_interface::srv::BasicSrv>::SharedFuture inner_future)
        {
            RCLCPP_INFO(this->get_logger(), "INNER LABEL");
            auto result = inner_future.get();
            b = result->a;
            RCLCPP_INFO(this->get_logger(), "[inner service] callback executed");
        };
        RCLCPP_INFO(this->get_logger(), "OUTER LABEL");
        auto inner_future_result = cli_->async_send_request(inner_request, inner_client_callback);
        RCLCPP_INFO(this->get_logger(), "[outer timer] callback executed: b = %d", b);
    }

    void wait()
    {
        while (!cli_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                exit(1);
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    auto node = std::make_shared<BasicClient>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}