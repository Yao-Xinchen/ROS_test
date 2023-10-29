#include "rclcpp/rclcpp.hpp"

#include "test_interface/msg/goal_int.hpp"

class Trigger : public rclcpp::Node
{
public:
    Trigger() : Node("trigger")
    {
        count = 0;
        timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&Trigger::timer_callback, this));
        pub_ = this->create_publisher<test_interface::msg::GoalInt>("goal_int", 10);
    }

private:
    int count;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<test_interface::msg::GoalInt>::SharedPtr pub_;

    void timer_callback()
    {
        auto msg = test_interface::msg::GoalInt();
        msg.goal = count;
        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "change to: %d", count);
        count++;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Trigger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}