#include "dm_test/can_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <linux/can.h>
#include <sys/types.h>

void set_on(can_frame& frame)
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

void set_off(can_frame& frame)
{
    frame.data[0] = 0xff;
    frame.data[1] = 0xff;
    frame.data[2] = 0xff;
    frame.data[3] = 0xff;
    frame.data[4] = 0xff;
    frame.data[5] = 0xff;
    frame.data[6] = 0xff;
    frame.data[7] = 0xfd;
}

void set_zero(can_frame& frame)
{
    frame.data[0] = 0xff;
    frame.data[1] = 0xff;
    frame.data[2] = 0xff;
    frame.data[3] = 0xff;
    frame.data[4] = 0xff;
    frame.data[5] = 0xff;
    frame.data[6] = 0xff;
    frame.data[7] = 0xfe;
}

CanDriver* can_driver_;
can_frame tx_frame;

enum Utility
{
    ON = 1,
    OFF = -1,
    ZERO = 0
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc != 2)
    {
        printf("Usage: dm_basic <utility>\n");
        printf("Utility options: 1. turn on 0. set zero -1. turn off\n");
        return 1;
    }

    can_driver_ = new CanDriver(0);

    tx_frame.can_id = 0x001;
    tx_frame.can_dlc = 8;
    
    Utility u = (Utility)atoi(argv[1]);
    switch (u)
    {
        case ON:
            set_on(tx_frame);
            break;
        case OFF:
            set_off(tx_frame);
            break;
        case ZERO:
            set_zero(tx_frame);
            break;
        default:
            printf("Invalid utility option\n");
            return 1;
    }

    can_driver_->send_frame(tx_frame);

    delete can_driver_;

    rclcpp::shutdown();
    return 0;
}
