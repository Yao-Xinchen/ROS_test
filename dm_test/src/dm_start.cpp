#include "dm_test/dm_driver.h"
#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <linux/can.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    can_frame tx_frame;
    tx_frame.can_id = 0x001;
    tx_frame.can_dlc = 8;
    auto can_driver_ = std::make_shared<CanDriver>();
    tx_frame.data[0] = 0xff;
    tx_frame.data[1] = 0xff;
    tx_frame.data[2] = 0xff;
    tx_frame.data[3] = 0xff;
    tx_frame.data[4] = 0xff;
    tx_frame.data[5] = 0xff;
    tx_frame.data[6] = 0xff;
    tx_frame.data[7] = 0xfc;
    for (int i = 10; i > 0; i --) {
        can_driver_->send_frame(tx_frame);
        printf("send frame: %d\n", i);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    rclcpp::shutdown();
    return 0;
}