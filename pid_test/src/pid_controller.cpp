#include "rclcpp/rclcpp.hpp"
#include "pid_test/can_driver.hpp"

class PidFeedback : public rclcpp::Node
{
    PidFeedback() : Node("pid_feedback")
    {

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