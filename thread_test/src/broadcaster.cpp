#include "rclcpp/rclcpp.hpp"

#include "test_interface/msg/goal_int.hpp"

class Broadcaster : public rclcpp::Node
{
public:
        Broadcaster() : Node("broadcaster")
        {
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Broadcaster::timer_callback, this));
            sub_ = this->create_subscription<test_interface::msg::GoalInt>("goal_int", 10, [this](test_interface::msg::GoalInt::SharedPtr msg) {
                number = msg->goal;
            });
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<test_interface::msg::GoalInt>::SharedPtr sub_;
        int number = 0;

    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "number: %d", number);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("broadcaster");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}