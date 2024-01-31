#include "rclcpp/rclcpp.hpp"
#include "unitree_test/unitree_driver.hpp"
#include <memory>
#include <rclcpp/timer.hpp>

class UnitreeTest : public rclcpp::Node
{
public:
    UnitreeTest() : Node("unitree_controller")
    {
        unitree_driver_ = std::make_unique<UnitreeDriver>();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&UnitreeTest::timer_callback, this));
    }

private:
    std::unique_ptr<UnitreeDriver> unitree_driver_;
    rclcpp::TimerBase::SharedPtr timer_;
    float angle = 0.0;

    void timer_callback()
    {
        angle += 3.14 / 4;
        unitree_driver_->set_goal(angle, 0.0);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnitreeTest>());
    rclcpp::shutdown();
    return 0;
}