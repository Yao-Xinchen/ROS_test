#include "rclcpp/rclcpp.hpp"
#include <memory>

class RemoteTest : public rclcpp::Node
{
public:
    RemoteTest() : Node("remote_test")
    {

    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RemoteTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}