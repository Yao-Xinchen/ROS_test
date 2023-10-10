#include "rclcpp/rclcpp.hpp"
#include "can_test/can_driver.hpp"
#include "can_interface/msg/can_frame.hpp"
#include <linux/can.h>

class CanInterpreter: public rclcpp::Node 
{
public:
    CanInterpreter() : Node("can_interpreter")
    {
        sub_ = this->create_subscription<can_interface::msg::CanFrame>(
            "motor_goal", 10, [this](can_interface::msg::CanFrame::SharedPtr msg){
                this->msg_callback(*msg);
            });
        can_driver_ = new CanDriver(0);
    }

    ~CanInterpreter()
    {
        delete can_driver_;
    }

private:
    rclcpp::Subscription<can_interface::msg::CanFrame>::SharedPtr sub_;
    CanDriver* can_driver_;
    can_frame tx_frame;

    void msg_callback(const can_interface::msg::CanFrame & msg)
    {
        tx_frame.can_id = msg.id;
        tx_frame.can_dlc = msg.dlc;
        for (int i = 0; i < msg.dlc; i++)
        {
            tx_frame.data[i] = msg.data[i];
        }
        can_driver_->send_frame(tx_frame);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanInterpreter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}