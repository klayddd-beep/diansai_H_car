// Laser GPIO driver with a fail-safe timeout and active-level configuration.
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class LaserGpioDriver : public rclcpp::Node
{
public:
  LaserGpioDriver()
  : Node("laser_gpio_driver")
  {
    declare_parameter<bool>("mock_mode", true);
    declare_parameter<double>("max_on_s", 3.0);
    declare_parameter<int>("gpio_number", 40);
    declare_parameter<bool>("active_low", true);
    declare_parameter<std::string>("gpio_command", "/usr/bin/gpio");
    declare_parameter<bool>("startup_on", false);

    mock_ = get_parameter("mock_mode").as_bool();
    max_on_s_ = get_parameter("max_on_s").as_double();
    gpio_number_ = get_parameter("gpio_number").as_int();
    active_low_ = get_parameter("active_low").as_bool();
    gpio_command_ = get_parameter("gpio_command").as_string();
    const bool startup_on = get_parameter("startup_on").as_bool();

    if (max_on_s_ <= 0.0) {
      throw std::invalid_argument("max_on_s must be greater than zero");
    }
    if (gpio_number_ < 0) {
      throw std::invalid_argument("gpio_number must not be negative");
    }

    status_ = create_publisher<std_msgs::msg::String>("/laser_status", 10);

    if (!mock_) {
      initialise_gpio();
    }
    set(false, "ready");

    sub_ = create_subscription<std_msgs::msg::String>(
      "/laser_command", 10,
      [this](std_msgs::msg::String::SharedPtr message) {command(message->data);});
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() {
        if (on_ && now() >= off_at_) {
          set(false, "timeout_off");
        }
      });

    if (startup_on) {
      off_at_ = now() + rclcpp::Duration::from_seconds(max_on_s_);
      set(true, "on");
    }
  }

  ~LaserGpioDriver() override
  {
    // Keep PB0 actively at its OFF level on an orderly ROS shutdown.
    if (!mock_ && gpio_ready_) {
      write_gpio(false);
    }
  }

private:
  void command(const std::string & command)
  {
    if (command == "ON") {
      off_at_ = now() + rclcpp::Duration::from_seconds(max_on_s_);
      set(true, "on");
    } else if (command == "OFF") {
      set(false, "off");
    } else {
      RCLCPP_WARN(get_logger(), "ignoring unknown laser command: '%s'", command.c_str());
    }
  }

  void initialise_gpio()
  {
    // wiringPi's setuid gpio helper exports the sysfs line and atomically applies
    // the initial level. This avoids a short active-low pulse while changing PB0
    // from input to output. `gpio export` always uses the Linux GPIO number.
    const std::string gpio = std::to_string(gpio_number_);
    const char * initial_level = active_low_ ? "high" : "low";
    const pid_t pid = fork();
    if (pid < 0) {
      throw std::runtime_error("failed to fork gpio export helper");
    }
    if (pid == 0) {
      execl(
        gpio_command_.c_str(), gpio_command_.c_str(), "export", gpio.c_str(),
        initial_level, static_cast<char *>(nullptr));
      _exit(127);
    }

    int status = 0;
    if (waitpid(pid, &status, 0) < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
      throw std::runtime_error("gpio export helper failed for GPIO " + gpio);
    }

    gpio_value_path_ = "/sys/class/gpio/gpio" + gpio + "/value";
    gpio_ready_ = true;
    RCLCPP_INFO(
      get_logger(), "using Linux GPIO %d (%s active), OFF level=%d",
      gpio_number_, active_low_ ? "low" : "high", off_level());
  }

  int off_level() const
  {
    return active_low_ ? 1 : 0;
  }

  bool write_gpio(bool on)
  {
    const int electrical_level = on ? 1 - off_level() : off_level();
    std::ofstream value(gpio_value_path_);
    if (!value) {
      RCLCPP_ERROR(get_logger(), "cannot open GPIO value file: %s", gpio_value_path_.c_str());
      return false;
    }
    value << electrical_level;
    value.flush();
    if (!value) {
      RCLCPP_ERROR(get_logger(), "cannot write GPIO value file: %s", gpio_value_path_.c_str());
      return false;
    }
    return true;
  }

  void publish_status(const std::string & state)
  {
    std_msgs::msg::String message;
    message.data = state;
    status_->publish(message);
  }

  void set(bool on, const char * state)
  {
    if (!mock_ && !write_gpio(on)) {
      // If an OFF write fails, retain the previous ON state so the 50 ms timer
      // keeps retrying the safe transition instead of silently giving up.
      if (on) {
        on_ = false;
      }
      publish_status("error");
      return;
    }
    on_ = on;
    publish_status(state);
    RCLCPP_INFO(get_logger(), "laser %s%s", state, mock_ ? " (mock)" : "");
  }

  bool mock_{true};
  bool on_{false};
  bool active_low_{true};
  bool gpio_ready_{false};
  double max_on_s_{3.0};
  int gpio_number_{40};
  std::string gpio_command_;
  std::string gpio_value_path_;
  rclcpp::Time off_at_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LaserGpioDriver>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("laser_gpio_driver"), "%s", error.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
