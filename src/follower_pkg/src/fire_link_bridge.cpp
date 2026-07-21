// Aircraft/car UDP link used on the car side.
// Packet format is little-endian and fixed at 32 bytes; see the communication
// agreement in the workspace root.  The target boards are little-endian ARM64.
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace
{
constexpr uint16_t kMagic = 0xF14E;
constexpr uint8_t kTelemetry = 1;
constexpr uint8_t kCarStart = 3;
constexpr uint8_t kUnknownPhase = 255;

struct __attribute__((packed)) FireLinkPacket
{
  uint16_t magic;
  uint8_t type;
  uint8_t phase;
  uint16_t seq;
  uint16_t reserved16;
  uint32_t stamp_ms;
  float x_dm;
  float y_dm;
  float distance_dm;
  float height_dm;
  uint32_t reserved32;
};
static_assert(sizeof(FireLinkPacket) == 32, "FireLinkPacket must be 32 bytes");

uint32_t monotonic_ms()
{
  using namespace std::chrono;
  return static_cast<uint32_t>(
    duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}
}  // namespace

class FireLinkBridge : public rclcpp::Node
{
public:
  FireLinkBridge()
  : Node("fire_link_bridge")
  {
    declare_parameter<int>("telemetry_udp_port", 8892);
    declare_parameter<std::string>("drone_ip", "");
    declare_parameter<std::string>("fallback_broadcast_ip", "10.42.0.255");
    declare_parameter<int>("start_udp_port", 8893);
    declare_parameter<int>("start_redundancy", 5);
    declare_parameter<int>("start_interval_ms", 5);
    declare_parameter<std::string>("button_gpio_value_path", "");
    declare_parameter<bool>("button_active_low", true);
    declare_parameter<int>("button_debounce_ms", 50);

    telemetry_port_ = get_parameter("telemetry_udp_port").as_int();
    start_port_ = get_parameter("start_udp_port").as_int();
    redundancy_ = std::max(1, static_cast<int>(get_parameter("start_redundancy").as_int()));
    start_interval_ms_ =
      std::max(1, static_cast<int>(get_parameter("start_interval_ms").as_int()));
    gpio_path_ = get_parameter("button_gpio_value_path").as_string();
    drone_ip_ = get_parameter("drone_ip").as_string();
    fallback_broadcast_ip_ = get_parameter("fallback_broadcast_ip").as_string();
    active_low_ = get_parameter("button_active_low").as_bool();
    debounce_ms_ =
      std::max(0, static_cast<int>(get_parameter("button_debounce_ms").as_int()));
    if (telemetry_port_ < 1 || telemetry_port_ > 65535 ||
      start_port_ < 1 || start_port_ > 65535)
    {
      throw std::runtime_error("UDP ports must be in the range 1..65535");
    }

    rx_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    tx_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (rx_fd_ < 0 || tx_fd_ < 0) {
      throw std::runtime_error("cannot create fire-link UDP sockets");
    }

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(static_cast<uint16_t>(telemetry_port_));
    if (::bind(rx_fd_, reinterpret_cast<sockaddr *>(&local), sizeof(local)) < 0) {
      throw std::runtime_error("cannot bind telemetry UDP port " + std::to_string(telemetry_port_));
    }
    ::fcntl(rx_fd_, F_SETFL, ::fcntl(rx_fd_, F_GETFL, 0) | O_NONBLOCK);

    if (!drone_ip_.empty()) {
      if (::inet_pton(AF_INET, drone_ip_.c_str(), &parameter_destination_) != 1) {
        throw std::runtime_error("invalid drone_ip: " + drone_ip_);
      }
      have_parameter_destination_ = true;
    }
    if (!fallback_broadcast_ip_.empty()) {
      if (::inet_pton(
          AF_INET, fallback_broadcast_ip_.c_str(), &broadcast_destination_) != 1)
      {
        throw std::runtime_error(
                "invalid fallback_broadcast_ip: " + fallback_broadcast_ip_);
      }
      int enable_broadcast = 1;
      if (::setsockopt(
          tx_fd_, SOL_SOCKET, SO_BROADCAST, &enable_broadcast,
          sizeof(enable_broadcast)) < 0)
      {
        throw std::runtime_error(
                "cannot enable UDP broadcast: " + std::string(std::strerror(errno)));
      }
      have_broadcast_destination_ = true;
    }

    telemetry_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/drone_telemetry", 10);
    start_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/drone_start", 10, [this](std_msgs::msg::Empty::SharedPtr) {request_start("ROS topic");});
    button_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/drone_start_button", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !topic_button_pressed_) {
          request_start("button topic");
        }
        topic_button_pressed_ = msg->data;
      });

    poll_timer_ = create_wall_timer(
      std::chrono::milliseconds(20), [this]() {
        poll_telemetry();
        poll_gpio_button();
      });
    send_timer_ = create_wall_timer(
      std::chrono::milliseconds(start_interval_ms_), [this]() {send_pending_start();});

    RCLCPP_INFO(
      get_logger(),
      "listening for aircraft telemetry on UDP :%d; CAR_START resolution: "
      "learned sender, then drone_ip='%s', then fallback_broadcast_ip='%s'; port %d",
      telemetry_port_, drone_ip_.c_str(), fallback_broadcast_ip_.c_str(), start_port_);
    if (gpio_path_.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "GPIO button disabled; publish std_msgs/Empty on /drone_start or Bool on /drone_start_button");
    } else {
      RCLCPP_INFO(get_logger(), "polling start button at %s", gpio_path_.c_str());
    }
  }

  ~FireLinkBridge() override
  {
    if (rx_fd_ >= 0) {
      ::close(rx_fd_);
    }
    if (tx_fd_ >= 0) {
      ::close(tx_fd_);
    }
  }

private:
  void poll_telemetry()
  {
    while (true) {
      // Keep one extra byte so an oversized datagram cannot be silently
      // truncated to sizeof(FireLinkPacket) and accepted as a valid packet.
      std::array<uint8_t, sizeof(FireLinkPacket) + 1> buffer{};
      sockaddr_in sender{};
      socklen_t sender_size = sizeof(sender);
      const auto size = ::recvfrom(
        rx_fd_, buffer.data(), buffer.size(), 0,
        reinterpret_cast<sockaddr *>(&sender), &sender_size);
      if (size < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "telemetry recvfrom failed: %s",
            std::strerror(errno));
        }
        return;
      }
      if (size != static_cast<ssize_t>(sizeof(FireLinkPacket))) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "discarding telemetry packet with invalid length: %zd", size);
        continue;
      }
      FireLinkPacket packet{};
      std::memcpy(&packet, buffer.data(), sizeof(packet));
      if (packet.magic != kMagic ||
        packet.type != kTelemetry)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "discarding invalid telemetry packet");
        continue;
      }
      if (!std::isfinite(packet.x_dm) || !std::isfinite(packet.y_dm) ||
        !std::isfinite(packet.distance_dm) || !std::isfinite(packet.height_dm))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "discarding telemetry packet containing NaN/Inf");
        continue;
      }
      // A signed 16-bit delta handles normal uint16 sequence wraparound.
      if (have_telemetry_seq_ && static_cast<int16_t>(packet.seq - last_telemetry_seq_) <= 0) {
        continue;
      }
      last_telemetry_seq_ = packet.seq;
      have_telemetry_seq_ = true;
      learned_destination_ = sender.sin_addr;
      have_learned_destination_ = true;
      std_msgs::msg::Float32MultiArray msg;
      msg.layout.dim.resize(1);
      msg.layout.dim[0].label = "x_dm,y_dm,distance_dm,height_dm,phase,seq";
      msg.layout.dim[0].size = 6;
      msg.layout.dim[0].stride = 6;
      msg.data = {
        packet.x_dm, packet.y_dm, packet.distance_dm, packet.height_dm,
        static_cast<float>(packet.phase), static_cast<float>(packet.seq)};
      telemetry_pub_->publish(msg);
    }
  }

  void request_start(const char * source)
  {
    if (pending_start_packets_ > 0) {
      RCLCPP_INFO(get_logger(), "start burst already in progress; ignoring %s trigger", source);
      return;
    }
    start_packet_ = {};
    start_packet_.magic = kMagic;
    start_packet_.type = kCarStart;
    start_packet_.phase = kUnknownPhase;
    start_packet_.seq = start_seq_++;
    start_packet_.stamp_ms = monotonic_ms();
    pending_start_packets_ = redundancy_;
    RCLCPP_INFO(
      get_logger(), "start requested by %s: sending seq=%u x%d", source, start_packet_.seq,
      redundancy_);
    // Send the first copy immediately; the timer sends the remaining copies.
    send_pending_start();
  }

  void send_pending_start()
  {
    if (pending_start_packets_ <= 0) {
      return;
    }
    sockaddr_in destination{};
    destination.sin_family = AF_INET;
    destination.sin_port = htons(static_cast<uint16_t>(start_port_));
    const char * destination_source = nullptr;
    if (have_learned_destination_) {
      destination.sin_addr = learned_destination_;
      destination_source = "learned";
    } else if (have_parameter_destination_) {
      destination.sin_addr = parameter_destination_;
      destination_source = "param";
    } else if (have_broadcast_destination_) {
      destination.sin_addr = broadcast_destination_;
      destination_source = "broadcast";
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "cannot send CAR_START: no learned sender, drone_ip, or fallback_broadcast_ip");
      --pending_start_packets_;
      return;
    }
    char destination_ip[INET_ADDRSTRLEN]{};
    if (::inet_ntop(
        AF_INET, &destination.sin_addr, destination_ip, sizeof(destination_ip)) == nullptr)
    {
      std::strncpy(destination_ip, "<invalid>", sizeof(destination_ip) - 1);
    }
    const int copy_number = redundancy_ - pending_start_packets_ + 1;
    RCLCPP_INFO(
      get_logger(), "sending CAR_START seq=%u copy=%d/%d to %s:%d source=%s",
      start_packet_.seq, copy_number, redundancy_, destination_ip, start_port_,
      destination_source);
    const auto sent = ::sendto(
      tx_fd_, &start_packet_, sizeof(start_packet_), 0,
      reinterpret_cast<const sockaddr *>(&destination), sizeof(destination));
    if (sent != static_cast<ssize_t>(sizeof(start_packet_))) {
      RCLCPP_ERROR(get_logger(), "failed to send CAR_START packet: %s", std::strerror(errno));
    }
    --pending_start_packets_;
  }

  bool read_gpio_level(bool & level)
  {
    std::ifstream value(gpio_path_);
    char c = 0;
    if (!(value >> c) || (c != '0' && c != '1')) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "cannot read GPIO button value from %s",
        gpio_path_.c_str());
      return false;
    }
    level = c == '1';
    return true;
  }

  void poll_gpio_button()
  {
    if (gpio_path_.empty()) {
      return;
    }
    bool electrical_level = false;
    if (!read_gpio_level(electrical_level)) {
      return;
    }
    const bool pressed = active_low_ ? !electrical_level : electrical_level;
    const auto now_time = std::chrono::steady_clock::now();
    if (!gpio_initialized_) {
      gpio_initialized_ = true;
      gpio_candidate_ = pressed;
      gpio_stable_ = pressed;
      gpio_candidate_since_ = now_time;
      return;
    }
    if (pressed != gpio_candidate_) {
      gpio_candidate_ = pressed;
      gpio_candidate_since_ = now_time;
      return;
    }
    const auto stable_for =
      std::chrono::duration_cast<std::chrono::milliseconds>(
      now_time -
      gpio_candidate_since_).count();
    if (gpio_candidate_ != gpio_stable_ && stable_for >= debounce_ms_) {
      gpio_stable_ = gpio_candidate_;
      if (gpio_stable_) {
        request_start("GPIO button");
      }
    }
  }

  int rx_fd_{-1};
  int tx_fd_{-1};
  int telemetry_port_{8892};
  int start_port_{8893};
  int redundancy_{5};
  int start_interval_ms_{5};
  int debounce_ms_{50};
  bool active_low_{true};
  bool have_telemetry_seq_{false};
  bool have_learned_destination_{false};
  bool have_parameter_destination_{false};
  bool have_broadcast_destination_{false};
  bool topic_button_pressed_{false};
  bool gpio_initialized_{false};
  bool gpio_candidate_{false};
  bool gpio_stable_{false};
  uint16_t last_telemetry_seq_{0};
  uint16_t start_seq_{0};
  int pending_start_packets_{0};
  std::string gpio_path_;
  std::string drone_ip_;
  std::string fallback_broadcast_ip_;
  std::chrono::steady_clock::time_point gpio_candidate_since_{};
  in_addr learned_destination_{};
  in_addr parameter_destination_{};
  in_addr broadcast_destination_{};
  FireLinkPacket start_packet_{};
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr telemetry_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_sub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr send_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FireLinkBridge>());
  rclcpp::shutdown();
  return 0;
}
