// GPIO abstraction. `mock_mode=true` is deliberate until the laser wiring and pin are confirmed.
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
class LaserGpioDriver : public rclcpp::Node {
public:
  LaserGpioDriver() : Node("laser_gpio_driver") {
    declare_parameter<bool>("mock_mode", true); declare_parameter<double>("max_on_s", 3.0);
    mock_ = get_parameter("mock_mode").as_bool(); max_on_s_ = get_parameter("max_on_s").as_double();
    status_ = create_publisher<std_msgs::msg::String>("/laser_status", 10);
    sub_ = create_subscription<std_msgs::msg::String>("/laser_command", 10, [this](std_msgs::msg::String::SharedPtr m) { command(m->data); });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { if (on_ && now() > off_at_) set(false, "timeout_off"); });
    set(false, "ready");
  }
private:
  void command(const std::string & c) { if (c == "ON") { off_at_ = now() + rclcpp::Duration::from_seconds(max_on_s_); set(true, "on"); } else if (c == "OFF") set(false, "off"); }
  void set(bool on, const char * state) { on_ = on; /* Hardware stage: replace this branch with libgpiod set_value(). */
    std_msgs::msg::String m; m.data = state; status_->publish(m); RCLCPP_INFO(get_logger(), "laser %s%s", state, mock_ ? " (mock)" : ""); }
  bool mock_{true}, on_{false}; double max_on_s_{3.0}; rclcpp::Time off_at_; rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_; rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_; rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv) { rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<LaserGpioDriver>()); rclcpp::shutdown(); }
