#include "rclcpp/rclcpp.hpp"
#include "test_interface/srv/basic_srv.hpp"

class BasicServer : public rclcpp::Node
{
public:
    BasicServer() : Node("basic_server")
    {
        RCLCPP_INFO(this->get_logger(), "Hello from %s", this->get_name());
        srv_ = this->create_service<test_interface::srv::BasicSrv>("basic_srv", std::bind(&BasicServer::callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    rclcpp::Service<test_interface::srv::BasicSrv>::SharedPtr srv_;
    void callback(const std::shared_ptr<test_interface::srv::BasicSrv::Request> request, std::shared_ptr<test_interface::srv::BasicSrv::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Request received.");
        response->a = 1;
        RCLCPP_INFO(this->get_logger(), "Response sent: %d", response->a);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BasicServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}