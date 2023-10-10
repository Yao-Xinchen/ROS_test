#include "rclcpp/rclcpp.hpp"
#include "can_test/can_driver.hpp"
#include "can_interface/msg/can_frame.hpp"
#include <cstdio>
#include <linux/can.h>
#include <rclcpp/timer.hpp>

class CanBroadcaster: public rclcpp::Node 
{
public:
    CanBroadcaster() : Node("can_broadcaster")
    {
        count = 0;
        can_driver_ = new CanDriver(0);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), [this](){
                tx_frame.can_id = 0x200;
                tx_frame.can_dlc = 8;
                for (int i = 0; i < 8; i++)
                {
                    tx_frame.data[i] = i;
                }
                can_driver_->send_frame(tx_frame);
                printf("send frame: %d\n", count);
                count++;
            });
    }

    ~CanBroadcaster()
    {
        delete can_driver_;
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    CanDriver* can_driver_;
    can_frame tx_frame;
    int count;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}