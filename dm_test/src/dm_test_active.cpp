#include "dm_test/dm_driver.h"
#include "rclcpp/rclcpp.hpp"

#define PI 3.14159265358979323846

class DmTestActive : public rclcpp::Node
{
public:
    DmTestActive() : Node("dm_test_active")
    {
        printf("node constructed\n");
        dm_driver_ = new DmMitDriver(1, 2, 0.1);
        dm_driver_->turn_on();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&DmTestActive::timer_callback, this));
    }

    ~DmTestActive()
    {
        dm_driver_->turn_off();
        delete dm_driver_;
        printf("node destructed\n");
    }

private:
    DmDriver* dm_driver_;
    rclcpp::TimerBase::SharedPtr timer_;
    float goal_pos = 0;
    void timer_callback()
    {
        goal_pos += PI / 4;
        goal_pos = fmod(goal_pos, 2 * PI);
        dm_driver_->set_position(goal_pos);
        printf("goal_pos: %f\n", goal_pos); 
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DmTestActive>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}