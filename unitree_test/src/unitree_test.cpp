#include "rclcpp/rclcpp.hpp"
#include "unitree_test/unitree_driver.hpp"
#include <memory>

class UnitreeTest : public rclcpp::Node
{
public:
    UnitreeTest() : Node("unitree_controller")
    {
        unitree_driver_ = std::make_unique<UnitreeDriver>();
    }

private:
    std::unique_ptr<UnitreeDriver> unitree_driver_; 
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnitreeTest>());
    rclcpp::shutdown();
    return 0;
}