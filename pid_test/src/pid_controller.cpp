#include "rclcpp/rclcpp.hpp"
#include "pid_test/can_driver.hpp"

#include "pid_test/motor_data.hpp"
#include "pid_test/motor_driver.hpp"

#include <can_interface/msg/detail/motor_goal__struct.hpp>
#include <can_interface/msg/motor_present.hpp>
#include <can_interface/msg/motor_goal.hpp>
#include <rclcpp/timer.hpp>

class PidController : public rclcpp::Node
{
public:
    PidController(Params params) : Node("pid_controller")
    {
        motor_driver_ = std::make_unique<MotorDriver>(2, params);
        frame_init();
        control_timer_ = this->create_wall_timer(std::chrono::milliseconds(CONTROL_R), std::bind(&PidController::control_timer_callback, this));
        feedback_timer_ = this->create_wall_timer(std::chrono::milliseconds(FEEDBACK_R), std::bind(&PidController::feedback_timer_callback, this));
        goal_sub_ = this->create_subscription<can_interface::msg::MotorGoal>("goal_vel", 10, std::bind(&PidController::goal_sub_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::TimerBase::SharedPtr control_timer_; // send control frame regularly
    rclcpp::TimerBase::SharedPtr feedback_timer_; // receive feedback frame regularly
    rclcpp::Subscription<can_interface::msg::MotorGoal>::SharedPtr goal_sub_; // receive goal velocity
    std::unique_ptr<MotorDriver> motor_driver_;

    void goal_sub_callback(const can_interface::msg::MotorGoal::SharedPtr msg)
    {
        motor_driver_->set_goal(msg->goal_vel, msg->goal_pos);
    }

    void control_timer_callback()
    {
        motor_driver_->write_frame();
        MotorDriver::send_frame();
    }

    void feedback_timer_callback()
    {
        MotorDriver::can_0->get_frame(MotorDriver::rx_frame);
        motor_driver_->process_rx();
    }

    void frame_init()
    {
        MotorDriver::tx_frame.can_id = 0x200;
        MotorDriver::tx_frame.can_dlc = 8;
        for (int i = 0; i < 8; i++) MotorDriver::tx_frame.data[i] = 0x00;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    Params params;

    if (argc != 9) {
        printf("Usage: %s vel pos p2v_kp p2v_ki p2v_kd v2c_kp v2c_ki v2c_kd\n", argv[0]);
        return 1;
    }

    // tested optimal params: kp 0.004, ki 0.00003, kd 0.1
    params.goal_vel = std::stof(argv[1]);
    params.goal_pos = std::stof(argv[2]);
    params.p2v_kp = std::stof(argv[3]);
    params.p2v_ki = std::stof(argv[4]);
    params.p2v_kd = std::stof(argv[5]);
    params.v2c_kp = std::stof(argv[6]);
    params.v2c_ki = std::stof(argv[7]);
    params.v2c_kd = std::stof(argv[8]);

    freopen("/home/robomaster/Plot/fb_data.txt", "w", stdout);

    auto node = std::make_shared<PidController>(params);
    rclcpp::spin(node);
    rclcpp::shutdown();

    fclose(stdout);
    return 0;
}