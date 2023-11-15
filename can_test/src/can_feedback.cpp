#include "rclcpp/rclcpp.hpp"
#include "can_test/can_driver.hpp"

#include "can_interface/srv/frame_present.hpp"

class CanFeedback : public rclcpp::Node
{
public:
    CanFeedback() : Node("can_feedback")
    {
        can_driver_ = new CanDriver(0);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&CanFeedback::timer_callback, this));
        srv_ = this->create_service<can_interface::srv::FramePresent>("frame_present", std::bind(&CanFeedback::srv_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

    ~CanFeedback()
    {
        delete can_driver_;
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<can_interface::srv::FramePresent>::SharedPtr srv_;

    CanDriver* can_driver_;
    can_frame rx_frame;

    void timer_callback()
    {
        can_driver_->get_frame(rx_frame);
    }

    void srv_callback(const std::shared_ptr<can_interface::srv::FramePresent::Request> request, std::shared_ptr<can_interface::srv::FramePresent::Response> response)
    {
        response->can_id = rx_frame.can_id;
        for (int i = 0; i < rx_frame.can_dlc; i++)
        {
            response->can_data[i] = rx_frame.data[i];
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanFeedback>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}