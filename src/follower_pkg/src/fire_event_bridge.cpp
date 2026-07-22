// UDP fire-event bridge. Packet is little-endian, fixed 16 bytes:
// [magic u16=0xFC11][seq u16][x_dm f32][y_dm f32][reserved f32].
// The bridge intentionally does not share a ROS DDS domain with the aircraft.
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{constexpr uint16_t kMagic = 0xFC11; struct __attribute__((packed)) Packet
{
  uint16_t magic, seq; float x_dm, y_dm, reserved;
}; static_assert(sizeof(Packet) == 16);}

class FireEventBridge : public rclcpp::Node
{
public:
  FireEventBridge()
  : Node("fire_event_bridge")
  {
    declare_parameter<int>("fire_udp_port", 8889); declare_parameter<int>("reply_port", 8890);
    declare_parameter<double>("seq_reset_timeout_s", 3.0);
    declare_parameter<int>("final_status_redundancy", 3);
    declare_parameter<int>("final_status_interval_ms", 50);
    port_ = get_parameter("fire_udp_port").as_int();
    reply_port_ = get_parameter("reply_port").as_int();
    seq_reset_timeout_s_ = get_parameter("seq_reset_timeout_s").as_double();
    final_status_redundancy_ = get_parameter("final_status_redundancy").as_int();
    final_status_interval_ms_ = get_parameter("final_status_interval_ms").as_int();
    if (!std::isfinite(seq_reset_timeout_s_) || seq_reset_timeout_s_ <= 0.0) {
      throw std::runtime_error("seq_reset_timeout_s must be finite and greater than zero");
    }
    if (final_status_redundancy_ < 1 || final_status_interval_ms_ < 0) {
      throw std::runtime_error("final status redundancy must be >= 1 and interval must be >= 0");
    }
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0); if (fd_ < 0) {
      throw std::runtime_error("cannot create UDP socket");
    }
    sockaddr_in a{}; a.sin_family = AF_INET; a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons(port_);
    if (::bind(fd_, reinterpret_cast<sockaddr *>(&a), sizeof(a)) < 0) {
      throw std::runtime_error("cannot bind fire UDP port");
    }
    ::fcntl(fd_, F_SETFL, ::fcntl(fd_, F_GETFL, 0) | O_NONBLOCK);
    event_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/fire_event", 10);
    status_sub_ = create_subscription<std_msgs::msg::String>(
      "/fire_mission_status", 10,
      [this](std_msgs::msg::String::SharedPtr s) {reply(s->data);});
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {poll();});
  }
  ~FireEventBridge() override {if (fd_ >= 0) {::close(fd_);}}

private:
  void poll()
  {
    std::array<uint8_t, sizeof(Packet) + 1> buffer{}; sockaddr_in from{};
    socklen_t nfrom = sizeof(from);
    while (true) {
      const auto n =
        ::recvfrom(
        fd_, buffer.data(), buffer.size(), 0, reinterpret_cast<sockaddr *>(&from),
        &nfrom);
      if (n < 0) {if (errno == EAGAIN || errno == EWOULDBLOCK) {return;} return;}
      if (n != sizeof(Packet)) {continue;}
      Packet p{}; std::memcpy(&p, buffer.data(), sizeof(p));
      if (p.magic != kMagic || !std::isfinite(p.x_dm) || !std::isfinite(p.y_dm)) {continue;}
      const auto now = std::chrono::steady_clock::now();
      if (have_seq_) {
        const double gap_s = std::chrono::duration<double>(now - last_packet_time_).count();
        if (gap_s > seq_reset_timeout_s_) {
          RCLCPP_INFO(
            get_logger(), "fire-event stream restarted (gap %.1fs), resetting sequence filter",
            gap_s);
          have_seq_ = false;
        }
      }
      if (have_seq_ && static_cast<int16_t>(p.seq - last_seq_) <= 0) {continue;}
      last_seq_ = p.seq; have_seq_ = true; sender_ = from.sin_addr; have_sender_ = true;
      last_packet_time_ = now;
      std_msgs::msg::Float32MultiArray msg; msg.data = {p.x_dm, p.y_dm, static_cast<float>(p.seq)};
      event_pub_->publish(msg);
    }
  }
  void reply(const std::string & text)
  {
    if (!have_sender_) {
      return;
    }
    sockaddr_in to{};
    to.sin_family = AF_INET;
    to.sin_addr = sender_;
    to.sin_port = htons(reply_port_);
    const bool terminal = text == "done" || text.rfind("failed:", 0) == 0;
    const int copies = terminal ? final_status_redundancy_ : 1;
    for (int copy = 0; copy < copies; ++copy) {
      const auto sent =
        ::sendto(
        fd_, text.c_str(), text.size(), 0, reinterpret_cast<sockaddr *>(&to), sizeof(to));
      if (sent != static_cast<ssize_t>(text.size())) {
        RCLCPP_WARN(
          get_logger(), "failed to send mission status '%s': %s", text.c_str(),
          std::strerror(errno));
      }
      if (copy + 1 < copies) {
        std::this_thread::sleep_for(std::chrono::milliseconds(final_status_interval_ms_));
      }
    }
  }
  int fd_{-1}, port_{8889}, reply_port_{8890}; uint16_t last_seq_{0};
  double seq_reset_timeout_s_{3.0};
  int final_status_redundancy_{3}, final_status_interval_ms_{50};
  bool have_seq_{false}, have_sender_{false}; in_addr sender_{};
  std::chrono::steady_clock::time_point last_packet_time_{};
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr event_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<FireEventBridge>()); rclcpp::shutdown();
}
