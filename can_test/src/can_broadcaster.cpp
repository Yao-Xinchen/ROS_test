#include "rclcpp/rclcpp.hpp"
#include "can_test/can_driver.hpp"
#include "can_interface/msg/can_frame.hpp"
#include <bits/stdint-uintn.h>
#include <cstdio>
#include <linux/can.h>
#include <rclcpp/timer.hpp>

#define T 10 // ms

class CanBroadcaster: public rclcpp::Node 
{
public:
    CanBroadcaster() : Node("can_broadcaster")
    {
        // count = 0;
        can_driver_ = new CanDriver(0);
        tx_frame.can_id = 0x001;
        tx_frame.can_dlc = 8;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(T), [this](){
                set_frame();
                can_driver_->send_frame(tx_frame);
                // printf("send frame: %d\n", count);
                // count++;
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
    // uint8_t count;
    void set_frame()
    {
        tx_frame.data[0] = 0xff;
        tx_frame.data[1] = 0xff;
        tx_frame.data[2] = 0xff;
        tx_frame.data[3] = 0xff;
        tx_frame.data[4] = 0xff;
        tx_frame.data[5] = 0xff;
        tx_frame.data[6] = 0xff;
        tx_frame.data[7] = 0xfc;
    
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}