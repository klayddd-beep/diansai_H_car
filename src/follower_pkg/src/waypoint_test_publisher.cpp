// 雷达定位闭环跑车测试节点(不依赖消防任务通信)。
//
// 维护一串 map 系航点,把当前航点发到 /target_position;用 TF map<-laser_link
// (Cartographer 提供)判断是否到达,到达后自动推进下一个。下游 diff_drive_controller
// 吃 /target_position 算差速 v/w 发 /cmd_vel,底盘桥转 $VW。等于车端的迷你
// RouteTargetPublisher,用来单独验证 雷达+建图+底盘 全链路,免去手动 ros2 topic pub。
//
// 航点用参数 waypoints 配置:扁平数组 [x0_cm,y0_cm,yaw0_deg, x1,y1,yaw1, ...]。
// 默认前进 1m 再原地掉头开回起点。loop=true 时到末点后循环。

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
double normalizeAngleDeg(double a)
{
  while (a > 180.0) {
    a -= 360.0;
  }
  while (a < -180.0) {
    a += 360.0;
  }
  return a;
}
}  // namespace

struct Waypoint
{
  double x_cm;
  double y_cm;
  double yaw_deg;
};

class WaypointTestPublisher : public rclcpp::Node
{
public:
  WaypointTestPublisher()
  : Node("waypoint_test_publisher"),
    current_idx_(0),
    started_(false),
    finished_(false)
  {
    // 默认航点:前进 1m -> 原地掉头开回起点。indoors 小范围,改 waypoints 参数即可。
    declare_parameter<std::vector<double>>(
      "waypoints", std::vector<double>{100.0, 0.0, 0.0, 0.0, 0.0, 180.0});
    declare_parameter<double>("pos_tol_cm", 10.0);   // 到达判定(应 >= diff_drive 的 pos_tol)
    declare_parameter<double>("yaw_tol_deg", 12.0);  // 到达判定(应 >= diff_drive 的 yaw_tol)
    declare_parameter<double>("publish_rate_hz", 10.0);
    declare_parameter<double>("start_delay_s", 3.0);  // 等 carto/TF 起来再开跑
    declare_parameter<bool>("loop", false);
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("laser_link_frame", "laser_link");

    pos_tol_cm_ = get_parameter("pos_tol_cm").as_double();
    yaw_tol_deg_ = get_parameter("yaw_tol_deg").as_double();
    const double rate_hz = get_parameter("publish_rate_hz").as_double();
    start_delay_s_ = get_parameter("start_delay_s").as_double();
    loop_ = get_parameter("loop").as_bool();
    map_frame_ = get_parameter("map_frame").as_string();
    laser_link_frame_ = get_parameter("laser_link_frame").as_string();

    const auto flat = get_parameter("waypoints").as_double_array();
    if (flat.size() < 3 || flat.size() % 3 != 0) {
      RCLCPP_FATAL(
        get_logger(),
        "waypoints 必须是 3 的倍数 [x_cm,y_cm,yaw_deg,...],当前 %zu 个数", flat.size());
      throw std::runtime_error("invalid waypoints parameter");
    }
    for (std::size_t i = 0; i + 2 < flat.size(); i += 3) {
      waypoints_.push_back(Waypoint{flat[i], flat[i + 1], flat[i + 2]});
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "/target_position", rclcpp::QoS(10));

    start_time_ = now();
    const double period_sec = 1.0 / std::max(rate_hz, 1.0);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_sec)),
      std::bind(&WaypointTestPublisher::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "waypoint_test_publisher up: %zu 个航点, loop=%s, start_delay=%.1fs, "
      "tol(pos=%.0fcm yaw=%.0fdeg)",
      waypoints_.size(), loop_ ? "true" : "false", start_delay_s_,
      pos_tol_cm_, yaw_tol_deg_);
  }

private:
  // 自身位姿:TF map<-laser_link(Cartographer 提供),与 diff_drive_controller 一致
  bool getCurrentPose(double & x_cm, double & y_cm, double & yaw_deg)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        map_frame_, laser_link_frame_, tf2::TimePointZero);
      x_cm = tf.transform.translation.x * 100.0;
      y_cm = tf.transform.translation.y * 100.0;
      tf2::Quaternion q;
      tf2::fromMsg(tf.transform.rotation, q);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      yaw_deg = yaw * 180.0 / M_PI;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF %s->%s unavailable: %s",
        map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool isReached(const Waypoint & wp, double x_cm, double y_cm, double yaw_deg) const
  {
    const double dxy = std::hypot(wp.x_cm - x_cm, wp.y_cm - y_cm);
    const double dyaw = std::fabs(normalizeAngleDeg(wp.yaw_deg - yaw_deg));
    return dxy <= pos_tol_cm_ && dyaw <= yaw_tol_deg_;
  }

  void publishWaypoint(const Waypoint & wp)
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data.resize(4);
    msg.data[0] = static_cast<float>(wp.x_cm);
    msg.data[1] = static_cast<float>(wp.y_cm);
    msg.data[2] = 0.0f;  // 地面车,z 不用
    msg.data[3] = static_cast<float>(wp.yaw_deg);
    target_pub_->publish(msg);
  }

  void timerCallback()
  {
    // 启动延时:给 carto 建图/TF 上线的时间,避免一上来就 TF 失败
    if (!started_) {
      if ((now() - start_time_).seconds() < start_delay_s_) {
        return;
      }
      started_ = true;
      RCLCPP_INFO(get_logger(), "开始跑航点 0/%zu", waypoints_.size());
    }

    const Waypoint & wp = waypoints_[current_idx_];
    // 持续发布当前航点,喂住 diff_drive_controller 的 target_timeout
    publishWaypoint(wp);

    if (finished_) {
      return;  // 不循环:停在末点保持(diff_drive 在容差内自然零速)
    }

    double x_cm = 0.0;
    double y_cm = 0.0;
    double yaw_deg = 0.0;
    if (!getCurrentPose(x_cm, y_cm, yaw_deg)) {
      return;  // 无定位:仅持续发当前点,不推进
    }

    if (isReached(wp, x_cm, y_cm, yaw_deg)) {
      RCLCPP_INFO(
        get_logger(),
        "到达航点 %zu (x=%.0f y=%.0f yaw=%.0f)",
        current_idx_, wp.x_cm, wp.y_cm, wp.yaw_deg);
      current_idx_++;
      if (current_idx_ >= waypoints_.size()) {
        if (loop_) {
          current_idx_ = 0;
          RCLCPP_INFO(get_logger(), "全部到达,loop 回第 0 个");
        } else {
          current_idx_ = waypoints_.size() - 1;  // 停在末点
          finished_ = true;
          RCLCPP_INFO(get_logger(), "全部航点完成,保持末点");
        }
      } else {
        RCLCPP_INFO(get_logger(), "推进到航点 %zu/%zu", current_idx_, waypoints_.size());
      }
    }
  }

  std::vector<Waypoint> waypoints_;
  std::size_t current_idx_;
  bool started_;
  bool finished_;

  double pos_tol_cm_;
  double yaw_tol_deg_;
  double start_delay_s_;
  bool loop_;
  std::string map_frame_;
  std::string laser_link_frame_;

  rclcpp::Time start_time_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointTestPublisher>());
  rclcpp::shutdown();
  return 0;
}
