// UDP fire-event bridge. Packet is little-endian, fixed 16 bytes:
// [magic u16=0xFC11][seq u16][x_dm f32][y_dm f32][reserved f32].
// The bridge intentionally does not share a ROS DDS domain with the aircraft.
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace { constexpr uint16_t kMagic = 0xFC11; struct __attribute__((packed)) Packet {
  uint16_t magic, seq; float x_dm, y_dm, reserved;
}; static_assert(sizeof(Packet) == 16); }

class FireEventBridge : public rclcpp::Node {
public:
  FireEventBridge() : Node("fire_event_bridge") {
    declare_parameter<int>("fire_udp_port", 8889); declare_parameter<int>("reply_port", 8890);
    port_ = get_parameter("fire_udp_port").as_int(); reply_port_ = get_parameter("reply_port").as_int();
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0); if (fd_ < 0) throw std::runtime_error("cannot create UDP socket");
    sockaddr_in a{}; a.sin_family = AF_INET; a.sin_addr.s_addr = htonl(INADDR_ANY); a.sin_port = htons(port_);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&a), sizeof(a)) < 0) throw std::runtime_error("cannot bind fire UDP port");
    ::fcntl(fd_, F_SETFL, ::fcntl(fd_, F_GETFL, 0) | O_NONBLOCK);
    event_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/fire_event", 10);
    status_sub_ = create_subscription<std_msgs::msg::String>("/fire_mission_status", 10,
      [this](std_msgs::msg::String::SharedPtr s) { reply(s->data); });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { poll(); });
  }
  ~FireEventBridge() override { if (fd_ >= 0) ::close(fd_); }
private:
  void poll() {
    std::array<uint8_t, sizeof(Packet) + 1> buffer{}; sockaddr_in from{}; socklen_t nfrom = sizeof(from);
    while (true) { const auto n = ::recvfrom(fd_, buffer.data(), buffer.size(), 0, reinterpret_cast<sockaddr*>(&from), &nfrom);
      if (n < 0) { if (errno == EAGAIN || errno == EWOULDBLOCK) return; return; }
      if (n != sizeof(Packet)) continue;
      Packet p{}; std::memcpy(&p, buffer.data(), sizeof(p));
      if (p.magic != kMagic || (have_seq_ && static_cast<int16_t>(p.seq-last_seq_) <= 0)) continue;
      last_seq_ = p.seq; have_seq_ = true; sender_ = from.sin_addr; have_sender_ = true;
      std_msgs::msg::Float32MultiArray msg; msg.data = {p.x_dm, p.y_dm, static_cast<float>(p.seq)}; event_pub_->publish(msg);
    }
  }
  void reply(const std::string & text) {
    if (!have_sender_) {
      return;
    }
    sockaddr_in to{};
    to.sin_family = AF_INET;
    to.sin_addr = sender_;
    to.sin_port = htons(reply_port_);
    ::sendto(fd_, text.c_str(), text.size(), 0, reinterpret_cast<sockaddr*>(&to), sizeof(to));
  }
  int fd_{-1}, port_{8889}, reply_port_{8890}; uint16_t last_seq_{0}; bool have_seq_{false}, have_sender_{false}; in_addr sender_{};
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr event_pub_; rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_; rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv) { rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<FireEventBridge>()); rclcpp::shutdown(); }
