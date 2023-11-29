#include "dm_test/dm_driver.h"
#include "rclcpp/rclcpp.hpp"

#include "test_interface/msg/goal_pos.hpp"

class DmTestPassive : public rclcpp::Node
{
public:
    DmTestPassive() : Node("dm_test_passive")
    {
        dm_driver_ = new DmMitDriver(1, 2, 0.1);
        dm_driver_->turn_on();
        sub_ = this->create_subscription<test_interface::msg::GoalPos>("goal_pos", 10, std::bind(&DmTestPassive::goal_pos_callback, this, std::placeholders::_1));
    }

    ~DmTestPassive()
    {
        dm_driver_->turn_off();
        delete dm_driver_;
    }

private:
    DmDriver* dm_driver_;
    rclcpp::Subscription<test_interface::msg::GoalPos>::SharedPtr sub_;

    void goal_pos_callback(const test_interface::msg::GoalPos::SharedPtr msg)
    {
        dm_driver_->set_position(msg->pos);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DmTestPassive>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}