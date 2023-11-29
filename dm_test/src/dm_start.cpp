#include "dm_test/can_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <linux/can.h>

void set_frame(can_frame& frame)
{
    frame.data[0] = 0xff;
    frame.data[1] = 0xff;
    frame.data[2] = 0xff;
    frame.data[3] = 0xff;
    frame.data[4] = 0xff;
    frame.data[5] = 0xff;
    frame.data[6] = 0xff;
    frame.data[7] = 0xfc;
}

CanDriver* can_driver_;
can_frame tx_frame; // must NOT be local variable, don't know why

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    can_driver_ = new CanDriver(0);

    tx_frame.can_id = 0x001;
    tx_frame.can_dlc = 8;
    set_frame(tx_frame);
    
    // for (int i = 10; i > 0; i --) {
    //     can_driver_->send_frame(tx_frame);
    //     // printf("send frame: %d\n", i);
    //     // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }

    can_driver_->send_frame(tx_frame);

    delete can_driver_;

    rclcpp::shutdown();
    return 0;
}
