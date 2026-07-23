// Fire mission state machine. Arena coordinates are dm; TF/control coordinates are m.
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "follower_pkg/msg/fire_detection.hpp"
#include "follower_pkg/msg/vision_servo_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
constexpr double kArenaWidthDm = 48.0;
constexpr double kArenaHeightDm = 40.0;

struct Point
{
  double x;
  double y;
};

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

class FireMissionManager : public rclcpp::Node
{
public:
  FireMissionManager()
  : Node("fire_mission_manager")
  {
    declare_parameter<double>("arena_origin_map_x_m", -0.25);
    declare_parameter<double>("arena_origin_map_y_m", 1.35);
    declare_parameter<double>("arena_origin_map_yaw_deg", -90.0);
    declare_parameter<double>("home_x_dm", 13.5);
    declare_parameter<double>("home_y_dm", 2.5);
    declare_parameter<double>("arrival_tol_dm", 1.2);
    declare_parameter<double>("district_match_tol_dm", 2.0);
    declare_parameter<double>("aim_tol_deg", 8.0);
    declare_parameter<double>("laser_on_s", 2.1);
    declare_parameter<double>("grid_dm", 1.0);
    declare_parameter<double>("safety_margin_dm", 1.5);
    declare_parameter<double>("pose_failure_timeout_s", 1.0);
    declare_parameter<double>("tf_max_age_s", 0.5);
    declare_parameter<double>("outside_arena_margin_dm", 3.0);
    declare_parameter<int>("vision_stable_frames", 8);
    declare_parameter<double>("vision_error_deadband_x", 0.06);
    declare_parameter<double>("vision_error_deadband_y", 0.06);
    declare_parameter<double>("vision_kp_v_mps", 0.08);
    declare_parameter<double>("vision_kp_w_rps", 0.60);
    declare_parameter<double>("vision_v_max_mps", 0.04);
    declare_parameter<double>("vision_w_max_rps", 0.20);
    declare_parameter<double>("vision_search_w_rps", 0.12);
    declare_parameter<double>("vision_search_yaw_deg", 15.0);
    declare_parameter<double>("vision_timeout_s", 12.0);
    declare_parameter<double>("vision_data_timeout_s", 0.5);
    declare_parameter<double>("vision_lost_grace_s", 0.3);
    declare_parameter<double>("vision_max_translation_dm", 2.0);
    declare_parameter<double>("vision_safety_lookahead_dm", 0.5);
    declare_parameter<std::vector<double>>(
      "obstacles_dm", std::vector<double>{});
    declare_parameter<std::vector<double>>(
      "district_stop_points_dm", std::vector<double>{});

    origin_x_m_ = get_parameter("arena_origin_map_x_m").as_double();
    origin_y_m_ = get_parameter("arena_origin_map_y_m").as_double();
    origin_yaw_rad_ =
      get_parameter("arena_origin_map_yaw_deg").as_double() * M_PI / 180.0;
    home_ = {
      get_parameter("home_x_dm").as_double(),
      get_parameter("home_y_dm").as_double()};
    arrival_tol_dm_ = get_parameter("arrival_tol_dm").as_double();
    district_match_tol_dm_ =
      get_parameter("district_match_tol_dm").as_double();
    aim_tol_rad_ = get_parameter("aim_tol_deg").as_double() * M_PI / 180.0;
    laser_on_s_ = get_parameter("laser_on_s").as_double();
    grid_dm_ = get_parameter("grid_dm").as_double();
    safety_margin_dm_ = get_parameter("safety_margin_dm").as_double();
    pose_failure_timeout_s_ =
      get_parameter("pose_failure_timeout_s").as_double();
    tf_max_age_s_ = get_parameter("tf_max_age_s").as_double();
    outside_arena_margin_dm_ =
      get_parameter("outside_arena_margin_dm").as_double();
    vision_stable_frames_required_ =
      get_parameter("vision_stable_frames").as_int();
    vision_error_deadband_x_ =
      get_parameter("vision_error_deadband_x").as_double();
    vision_error_deadband_y_ =
      get_parameter("vision_error_deadband_y").as_double();
    vision_kp_v_mps_ = get_parameter("vision_kp_v_mps").as_double();
    vision_kp_w_rps_ = get_parameter("vision_kp_w_rps").as_double();
    vision_v_max_mps_ = get_parameter("vision_v_max_mps").as_double();
    vision_w_max_rps_ = get_parameter("vision_w_max_rps").as_double();
    vision_search_w_rps_ = get_parameter("vision_search_w_rps").as_double();
    vision_search_yaw_rad_ =
      get_parameter("vision_search_yaw_deg").as_double() * M_PI / 180.0;
    vision_timeout_s_ = get_parameter("vision_timeout_s").as_double();
    vision_data_timeout_s_ =
      get_parameter("vision_data_timeout_s").as_double();
    vision_lost_grace_s_ = get_parameter("vision_lost_grace_s").as_double();
    vision_max_translation_dm_ =
      get_parameter("vision_max_translation_dm").as_double();
    vision_safety_lookahead_dm_ =
      get_parameter("vision_safety_lookahead_dm").as_double();
    obstacles_ = get_parameter("obstacles_dm").as_double_array();
    district_stop_points_ =
      get_parameter("district_stop_points_dm").as_double_array();
    validate_parameters();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    target_publisher_ =
      create_publisher<std_msgs::msg::Float32MultiArray>(
      "/target_position", 10);
    laser_publisher_ =
      create_publisher<std_msgs::msg::String>("/laser_command", 10);
    status_publisher_ =
      create_publisher<std_msgs::msg::String>("/fire_mission_status", 10);
    vision_servo_publisher_ =
      create_publisher<follower_pkg::msg::VisionServoCommand>(
      "/fire_vision/servo_command", 10);
    fire_subscription_ =
      create_subscription<std_msgs::msg::Float32MultiArray>(
      "/fire_event", 10,
      [this](std_msgs::msg::Float32MultiArray::SharedPtr message) {
        on_fire(*message);
      });
    laser_status_subscription_ =
      create_subscription<std_msgs::msg::String>(
      "/laser_status", 10,
      [this](std_msgs::msg::String::SharedPtr message) {
        laser_state_ = message->data;
      });
    reset_subscription_ = create_subscription<std_msgs::msg::Empty>(
      "/fire_mission_reset", 10,
      [this](std_msgs::msg::Empty::SharedPtr) {reset();});
    vision_detection_subscription_ =
      create_subscription<follower_pkg::msg::FireDetection>(
      "/fire_vision/detection", 10,
      [this](follower_pkg::msg::FireDetection::SharedPtr message) {
        on_vision_detection(*message);
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {tick();});
    report("ready");
  }

private:
  enum class State
  {
    IDLE, WAIT_POSE, DRIVE_FIRE, VISUAL_ALIGN, LASER_ON, DRIVE_HOME, SAFE_STOP
  };
  enum class PoseResult {OK, TF_LOST, OUTSIDE_ARENA};

  void validate_parameters() const
  {
    if (!std::isfinite(origin_x_m_) || !std::isfinite(origin_y_m_) ||
      !std::isfinite(origin_yaw_rad_))
    {
      throw std::invalid_argument("arena origin parameters must be finite");
    }
    if (!inside_arena(home_)) {
      throw std::invalid_argument("home position must be inside the arena");
    }
    if (!std::isfinite(arrival_tol_dm_) || arrival_tol_dm_ <= 0.0 ||
      !std::isfinite(district_match_tol_dm_) || district_match_tol_dm_ < 0.0 ||
      !std::isfinite(aim_tol_rad_) || aim_tol_rad_ <= 0.0 ||
      !std::isfinite(laser_on_s_) || laser_on_s_ <= 0.0 ||
      !std::isfinite(grid_dm_) || grid_dm_ <= 0.0 ||
      !std::isfinite(safety_margin_dm_) || safety_margin_dm_ < 0.0 ||
      !std::isfinite(pose_failure_timeout_s_) || pose_failure_timeout_s_ <= 0.0 ||
      !std::isfinite(tf_max_age_s_) || tf_max_age_s_ <= 0.0 ||
      !std::isfinite(outside_arena_margin_dm_) || outside_arena_margin_dm_ < 0.0 ||
      vision_stable_frames_required_ <= 0 ||
      !std::isfinite(vision_error_deadband_x_) ||
      vision_error_deadband_x_ <= 0.0 || vision_error_deadband_x_ >= 1.0 ||
      !std::isfinite(vision_error_deadband_y_) ||
      vision_error_deadband_y_ <= 0.0 || vision_error_deadband_y_ >= 1.0 ||
      !std::isfinite(vision_kp_v_mps_) || vision_kp_v_mps_ <= 0.0 ||
      !std::isfinite(vision_kp_w_rps_) || vision_kp_w_rps_ <= 0.0 ||
      !std::isfinite(vision_v_max_mps_) || vision_v_max_mps_ <= 0.0 ||
      !std::isfinite(vision_w_max_rps_) || vision_w_max_rps_ <= 0.0 ||
      !std::isfinite(vision_search_w_rps_) || vision_search_w_rps_ <= 0.0 ||
      !std::isfinite(vision_search_yaw_rad_) || vision_search_yaw_rad_ <= 0.0 ||
      vision_search_yaw_rad_ >= M_PI ||
      !std::isfinite(vision_timeout_s_) || vision_timeout_s_ <= 0.0 ||
      !std::isfinite(vision_data_timeout_s_) || vision_data_timeout_s_ <= 0.0 ||
      !std::isfinite(vision_lost_grace_s_) || vision_lost_grace_s_ < 0.0 ||
      vision_lost_grace_s_ >= vision_timeout_s_ ||
      !std::isfinite(vision_max_translation_dm_) ||
      vision_max_translation_dm_ <= 0.0 ||
      !std::isfinite(vision_safety_lookahead_dm_) ||
      vision_safety_lookahead_dm_ <= 0.0)
    {
      throw std::invalid_argument("mission distances and durations are invalid");
    }
    if (obstacles_.size() % 4 != 0 ||
      !std::all_of(
        obstacles_.begin(), obstacles_.end(),
        [](double value) {return std::isfinite(value);}))
    {
      throw std::invalid_argument(
              "obstacles_dm must contain finite xmin,ymin,xmax,ymax groups");
    }
    for (size_t index = 0; index < obstacles_.size(); index += 4) {
      if (obstacles_[index] >= obstacles_[index + 2] ||
        obstacles_[index + 1] >= obstacles_[index + 3])
      {
        throw std::invalid_argument(
                "each obstacle must satisfy xmin<xmax and ymin<ymax");
      }
    }
    if (district_stop_points_.size() != obstacles_.size() / 2 ||
      !std::all_of(
        district_stop_points_.begin(), district_stop_points_.end(),
        [](double value) {return std::isfinite(value);}))
    {
      throw std::invalid_argument(
              "district_stop_points_dm must contain one finite x,y pair per obstacle");
    }
    for (size_t index = 0; index < district_stop_points_.size(); index += 2) {
      const Point stop{
        district_stop_points_[index],
        district_stop_points_[index + 1]};
      if (!inside_arena(stop) || blocked(stop.x, stop.y)) {
        throw std::invalid_argument(
                "each district stop point must be inside the arena and outside inflated obstacles");
      }
    }
  }

  static bool inside_arena(const Point & point)
  {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           point.x >= 0.0 && point.x <= kArenaWidthDm &&
           point.y >= 0.0 && point.y <= kArenaHeightDm;
  }

  bool inside_arena_margin(const Point & point) const
  {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           point.x >= -outside_arena_margin_dm_ &&
           point.x <= kArenaWidthDm + outside_arena_margin_dm_ &&
           point.y >= -outside_arena_margin_dm_ &&
           point.y <= kArenaHeightDm + outside_arena_margin_dm_;
  }

  static Point clamp_to_arena(const Point & point)
  {
    return {
      std::clamp(point.x, 0.0, kArenaWidthDm),
      std::clamp(point.y, 0.0, kArenaHeightDm)};
  }

  PoseResult get_pose(Point & point, double & yaw_map)
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        "map", "laser_link", tf2::TimePointZero);
      last_tf_error_.clear();
      const rclcpp::Time transform_time(transform.header.stamp);
      const double transform_age_s = transform_time.nanoseconds() == 0 ?
        0.0 : std::max(0.0, (now() - transform_time).seconds());
      if (transform_time.nanoseconds() != 0 && transform_age_s > tf_max_age_s_) {
        last_tf_error_ = "latest transform is stale";
        // Count the outage from the last TF update, not from the later tick
        // that first noticed it was stale.
        pose_failure_initial_elapsed_s_ = transform_age_s;
        return PoseResult::TF_LOST;
      }
      pose_failure_initial_elapsed_s_ = 0.0;
      tf2::Quaternion quaternion;
      tf2::fromMsg(transform.transform.rotation, quaternion);
      double roll = 0.0;
      double pitch = 0.0;
      tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw_map);

      const double dx = transform.transform.translation.x - origin_x_m_;
      const double dy = transform.transform.translation.y - origin_y_m_;
      const double cosine = std::cos(origin_yaw_rad_);
      const double sine = std::sin(origin_yaw_rad_);
      point.x = (cosine * dx + sine * dy) * 10.0;
      point.y = (-sine * dx + cosine * dy) * 10.0;
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(yaw_map))
      {
        return PoseResult::TF_LOST;
      }
      if (!inside_arena_margin(point)) {
        return PoseResult::OUTSIDE_ARENA;
      }
      if (!inside_arena(point)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Pose (%.2f, %.2f) dm is outside the arena but within the %.2f dm margin; continuing",
          point.x, point.y, outside_arena_margin_dm_);
      }
      return PoseResult::OK;
    } catch (const tf2::TransformException & error) {
      last_tf_error_ = error.what();
      pose_failure_initial_elapsed_s_ = 0.0;
      return PoseResult::TF_LOST;
    }
  }

  const char * pose_failure_reason(PoseResult result) const
  {
    return result == PoseResult::OUTSIDE_ARENA ? "outside_arena" : "tf_lost";
  }

  bool tolerate_pose_failure(
    PoseResult result, const Point & observed_point, double observed_yaw_map)
  {
    const auto current_time = now();
    if (!pose_failure_active_) {
      pose_failure_active_ = true;
      pose_failure_started_at_ =
        current_time - rclcpp::Duration::from_seconds(pose_failure_initial_elapsed_s_);
    }
    const double elapsed_s = (current_time - pose_failure_started_at_).seconds();
    if (result == PoseResult::OUTSIDE_ARENA) {
      // TF itself is valid, so holding the observed pose stops without driving
      // back toward an older position.
      publish_target(observed_point, observed_yaw_map);
    } else if (have_last_valid_pose_) {
      publish_target(last_valid_pose_, last_valid_yaw_map_);
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Pose failure '%s' has lasted %.2f s (limit %.2f s); holding position%s%s",
      pose_failure_reason(result), elapsed_s, pose_failure_timeout_s_,
      last_tf_error_.empty() ? "" : ": ", last_tf_error_.c_str());
    if (elapsed_s >= pose_failure_timeout_s_) {
      fail(pose_failure_reason(result));
      return false;
    }
    return true;
  }

  void accept_pose(const Point & point, double yaw_map)
  {
    if (pose_failure_active_) {
      RCLCPP_INFO(get_logger(), "Pose recovered; resuming mission");
    }
    pose_failure_active_ = false;
    last_tf_error_.clear();
    last_valid_pose_ = point;
    last_valid_yaw_map_ = yaw_map;
    have_last_valid_pose_ = true;
  }

  bool blocked(double x, double y) const
  {
    if (x < 0.0 || x > kArenaWidthDm || y < 0.0 || y > kArenaHeightDm) {
      return true;
    }
    for (size_t index = 0; index + 3 < obstacles_.size(); index += 4) {
      if (x >= obstacles_[index] - safety_margin_dm_ &&
        x <= obstacles_[index + 2] + safety_margin_dm_ &&
        y >= obstacles_[index + 1] - safety_margin_dm_ &&
        y <= obstacles_[index + 3] + safety_margin_dm_)
      {
        return true;
      }
    }
    return false;
  }

  size_t district_for_fire(const Point & fire) const
  {
    for (size_t index = 0; index + 3 < obstacles_.size(); index += 4) {
      if (fire.x >= obstacles_[index] - district_match_tol_dm_ &&
        fire.x <= obstacles_[index + 2] + district_match_tol_dm_ &&
        fire.y >= obstacles_[index + 1] - district_match_tol_dm_ &&
        fire.y <= obstacles_[index + 3] + district_match_tol_dm_)
      {
        return index / 4;
      }
    }
    return kInvalidDistrict;
  }

  Point district_stop_point(size_t district) const
  {
    return {
      district_stop_points_[district * 2],
      district_stop_points_[district * 2 + 1]};
  }

  std::vector<Point> plan(Point start, Point goal) const
  {
    const int columns = static_cast<int>(kArenaWidthDm / grid_dm_) + 1;
    const int rows = static_cast<int>(kArenaHeightDm / grid_dm_) + 1;
    const auto cell_index = [columns](int x, int y) {
        return y * columns + x;
      };
    const auto to_cell = [this](Point point) {
        return std::pair<int, int>{
        static_cast<int>(std::round(point.x / grid_dm_)),
        static_cast<int>(std::round(point.y / grid_dm_))};
      };
    const auto start_cell = to_cell(start);
    const auto goal_cell = to_cell(goal);
    const int start_x = start_cell.first;
    const int start_y = start_cell.second;
    const int goal_x = goal_cell.first;
    const int goal_y = goal_cell.second;
    if (start_x < 0 || start_x >= columns || start_y < 0 || start_y >= rows ||
      goal_x < 0 || goal_x >= columns || goal_y < 0 || goal_y >= rows ||
      blocked(goal.x, goal.y))
    {
      return {};
    }

    std::vector<int> previous(columns * rows, -1);
    std::queue<int> pending;
    pending.push(cell_index(start_x, start_y));
    previous[cell_index(start_x, start_y)] = cell_index(start_x, start_y);
    constexpr int delta_x[4] = {1, -1, 0, 0};
    constexpr int delta_y[4] = {0, 0, 1, -1};
    while (!pending.empty()) {
      const int current = pending.front();
      pending.pop();
      const int x = current % columns;
      const int y = current / columns;
      if (x == goal_x && y == goal_y) {
        break;
      }
      for (int direction = 0; direction < 4; ++direction) {
        const int next_x = x + delta_x[direction];
        const int next_y = y + delta_y[direction];
        if (next_x >= 0 && next_x < columns &&
          next_y >= 0 && next_y < rows &&
          previous[cell_index(next_x, next_y)] < 0 &&
          !blocked(next_x * grid_dm_, next_y * grid_dm_))
        {
          previous[cell_index(next_x, next_y)] = current;
          pending.push(cell_index(next_x, next_y));
        }
      }
    }
    if (previous[cell_index(goal_x, goal_y)] < 0) {
      return {};
    }

    std::vector<Point> result;
    for (int current = cell_index(goal_x, goal_y);
      current != cell_index(start_x, start_y); current = previous[current])
    {
      result.push_back(
      {
        (current % columns) * grid_dm_,
        (current / columns) * grid_dm_});
    }
    result.push_back(start);
    std::reverse(result.begin(), result.end());
    result.back() = goal;
    return result;
  }

  void on_fire(const std_msgs::msg::Float32MultiArray & message)
  {
    if (message.data.size() < 2) {
      RCLCPP_WARN(get_logger(), "ignoring short /fire_event message");
      return;
    }
    if (state_ != State::IDLE) {
      report(current_status_);
      return;
    }
    const Point requested_fire{message.data[0], message.data[1]};
    if (!inside_arena(requested_fire)) {
      RCLCPP_WARN(get_logger(), "ignoring fire point outside the arena");
      report("failed:invalid_fire");
      return;
    }
    const size_t district = district_for_fire(requested_fire);
    if (district == kInvalidDistrict) {
      RCLCPP_WARN(
        get_logger(), "fire point (%.2f, %.2f) is not inside any configured district",
        requested_fire.x, requested_fire.y);
      report("failed:fire_not_in_district");
      return;
    }

    fire_point_ = requested_fire;
    fire_district_ = district;
    state_ = State::WAIT_POSE;
    tick();
  }

  void start_mission(const Point & current)
  {
    const Point stop = district_stop_point(fire_district_);
    const Point planning_start = clamp_to_arena(current);
    path_ = plan(planning_start, stop);
    if (path_.empty()) {
      fail("unreachable");
      return;
    }
    RCLCPP_INFO(
      get_logger(), "fire belongs to district %zu; driving to fixed stop (%.2f, %.2f) dm",
      fire_district_ + 1, stop.x, stop.y);
    waypoint_index_ = 0;
    state_ = State::DRIVE_FIRE;
    report("enroute");
  }

  void publish_target(Point point, double yaw_map)
  {
    const double cosine = std::cos(origin_yaw_rad_);
    const double sine = std::sin(origin_yaw_rad_);
    const double map_x =
      origin_x_m_ + (cosine * point.x - sine * point.y) / 10.0;
    const double map_y =
      origin_y_m_ + (sine * point.x + cosine * point.y) / 10.0;
    std_msgs::msg::Float32MultiArray message;
    message.data = {
      static_cast<float>(map_x * 100.0),
      static_cast<float>(map_y * 100.0),
      0.0F,
      static_cast<float>(yaw_map * 180.0 / M_PI)};
    target_publisher_->publish(message);
  }

  void on_vision_detection(const follower_pkg::msg::FireDetection & message)
  {
    if (state_ != State::VISUAL_ALIGN) {
      return;
    }
    vision_detection_seen_ = true;
    last_vision_detection_at_ = now();
    const bool valid =
      message.detected && message.image_width > 0 && message.image_height > 0 &&
      std::isfinite(message.error_x_norm) &&
      std::isfinite(message.error_y_norm) &&
      std::isfinite(message.area_ratio) && message.area_ratio > 0.0F;
    vision_detected_ = valid;
    if (!valid) {
      vision_stable_frames_ = 0;
      return;
    }

    vision_error_x_ = message.error_x_norm;
    vision_error_y_ = message.error_y_norm;
    vision_ever_detected_ = true;
    last_vision_target_at_ = last_vision_detection_at_;
    if (std::fabs(vision_error_x_) <= vision_error_deadband_x_ &&
      std::fabs(vision_error_y_) <= vision_error_deadband_y_)
    {
      ++vision_stable_frames_;
    } else {
      vision_stable_frames_ = 0;
    }
  }

  void publish_vision_servo(bool active, double linear_x, double angular_z)
  {
    follower_pkg::msg::VisionServoCommand message;
    message.header.stamp = now();
    message.header.frame_id = "base_link";
    message.active = active;
    message.linear_x_mps = static_cast<float>(linear_x);
    message.angular_z_rps = static_cast<float>(angular_z);
    vision_servo_publisher_->publish(message);
  }

  void publish_laser(const char * command)
  {
    std_msgs::msg::String message;
    message.data = command;
    laser_publisher_->publish(message);
  }

  void begin_laser()
  {
    publish_vision_servo(false, 0.0, 0.0);
    laser_state_.clear();
    publish_laser("ON");
    laser_started_at_ = now();
    state_ = State::LASER_ON;
    report("extinguishing");
  }

  void start_visual_align(const Point & current, double yaw_map)
  {
    state_ = State::VISUAL_ALIGN;
    vision_started_at_ = now();
    last_vision_detection_at_ = vision_started_at_;
    last_vision_target_at_ = vision_started_at_;
    vision_entry_yaw_map_ = yaw_map;
    vision_last_pose_ = current;
    vision_translation_dm_ = 0.0;
    vision_search_direction_ = 1.0;
    vision_detection_seen_ = false;
    vision_detected_ = false;
    vision_ever_detected_ = false;
    vision_stable_frames_ = 0;
    vision_error_x_ = 0.0;
    vision_error_y_ = 0.0;
    publish_vision_servo(true, 0.0, 0.0);
    RCLCPP_INFO(
      get_logger(),
      "Coarse fire pose reached; starting camera alignment (translation<=%.1f dm, yaw<=%.1f deg)",
      vision_max_translation_dm_, vision_search_yaw_rad_ * 180.0 / M_PI);
  }

  void tick_visual_align(const Point & current, double yaw_map)
  {
    vision_translation_dm_ +=
      std::hypot(current.x - vision_last_pose_.x, current.y - vision_last_pose_.y);
    vision_last_pose_ = current;
    if (vision_translation_dm_ > vision_max_translation_dm_) {
      fail("vision_motion_limit");
      return;
    }

    const auto current_time = now();
    const double elapsed_s = (current_time - vision_started_at_).seconds();
    if (elapsed_s >= vision_timeout_s_) {
      fail(vision_ever_detected_ ? "vision_timeout" : "vision_target_lost");
      return;
    }
    if (!vision_detection_seen_ ||
      (current_time - last_vision_detection_at_).seconds() >
      vision_data_timeout_s_)
    {
      if (elapsed_s >= vision_data_timeout_s_) {
        fail("vision_stale");
      } else {
        publish_vision_servo(true, 0.0, 0.0);
      }
      return;
    }

    if (vision_stable_frames_ >= vision_stable_frames_required_) {
      RCLCPP_INFO(
        get_logger(), "Camera alignment stable for %lld frames",
        static_cast<long long>(vision_stable_frames_));
      begin_laser();
      return;
    }

    const double yaw_from_entry =
      normalize_angle(yaw_map - vision_entry_yaw_map_);
    if (std::fabs(yaw_from_entry) >
      vision_search_yaw_rad_ + 2.0 * M_PI / 180.0)
    {
      fail("vision_yaw_limit");
      return;
    }

    if (!vision_detected_) {
      if ((current_time - last_vision_target_at_).seconds() <
        vision_lost_grace_s_)
      {
        publish_vision_servo(true, 0.0, 0.0);
        return;
      }
      if (vision_search_direction_ > 0.0 &&
        yaw_from_entry >= vision_search_yaw_rad_)
      {
        vision_search_direction_ = -1.0;
      } else if (vision_search_direction_ < 0.0 &&
        yaw_from_entry <= -vision_search_yaw_rad_)
      {
        vision_search_direction_ = 1.0;
      }
      publish_vision_servo(
        true, 0.0, vision_search_direction_ * vision_search_w_rps_);
      return;
    }

    double linear_x = 0.0;
    double angular_z = 0.0;
    if (std::fabs(vision_error_y_) > vision_error_deadband_y_) {
      linear_x = std::clamp(
        -vision_kp_v_mps_ * vision_error_y_,
        -vision_v_max_mps_, vision_v_max_mps_);
    }
    if (std::fabs(vision_error_x_) > vision_error_deadband_x_) {
      angular_z = std::clamp(
        -vision_kp_w_rps_ * vision_error_x_,
        -vision_w_max_rps_, vision_w_max_rps_);
    }

    if ((angular_z > 0.0 && yaw_from_entry >= vision_search_yaw_rad_) ||
      (angular_z < 0.0 && yaw_from_entry <= -vision_search_yaw_rad_))
    {
      fail("vision_yaw_limit");
      return;
    }
    if (linear_x != 0.0) {
      const double yaw_arena = yaw_map - origin_yaw_rad_;
      const double direction = linear_x > 0.0 ? 1.0 : -1.0;
      const Point lookahead{
        current.x + direction * vision_safety_lookahead_dm_ * std::cos(yaw_arena),
        current.y + direction * vision_safety_lookahead_dm_ * std::sin(yaw_arena)};
      if (blocked(lookahead.x, lookahead.y)) {
        fail("vision_unsafe_motion");
        return;
      }
    }
    publish_vision_servo(true, linear_x, angular_z);
  }

  void tick()
  {
    if (state_ == State::IDLE || state_ == State::SAFE_STOP) {
      return;
    }

    Point current{};
    double yaw_map = 0.0;
    const PoseResult pose_result = get_pose(current, yaw_map);
    if (pose_result != PoseResult::OK) {
      if (state_ == State::VISUAL_ALIGN) {
        publish_vision_servo(true, 0.0, 0.0);
      }
      tolerate_pose_failure(pose_result, current, yaw_map);
      return;
    }
    accept_pose(current, yaw_map);
    if (state_ == State::WAIT_POSE) {
      start_mission(current);
      return;
    }
    if (state_ == State::VISUAL_ALIGN) {
      tick_visual_align(current, yaw_map);
      return;
    }
    if (state_ == State::LASER_ON) {
      if (laser_state_ == "error") {
        fail("laser_gpio_error");
        return;
      }
      if (laser_state_ == "timeout_off") {
        fail("laser_timeout");
        return;
      }
      if ((now() - laser_started_at_).seconds() >= laser_on_s_) {
        publish_laser("OFF");
        path_ = plan(clamp_to_arena(current), home_);
        if (path_.empty()) {
          fail("home_unreachable");
          return;
        }
        waypoint_index_ = 0;
        state_ = State::DRIVE_HOME;
        report("returning");
      }
      return;
    }
    if (waypoint_index_ >= path_.size()) {
      if (state_ == State::DRIVE_FIRE) {
        start_visual_align(current, yaw_map);
      } else {
        state_ = State::IDLE;
        report("done");
      }
      return;
    }

    const Point target = path_[waypoint_index_];
    const double distance = std::hypot(current.x - target.x, current.y - target.y);
    const bool final_fire_waypoint =
      state_ == State::DRIVE_FIRE && waypoint_index_ + 1 == path_.size();
    if (final_fire_waypoint) {
      const double aim_yaw_map =
        std::atan2(
        fire_point_.y - current.y, fire_point_.x - current.x) +
        origin_yaw_rad_;
      publish_target(target, aim_yaw_map);
      if (distance < arrival_tol_dm_ &&
        std::fabs(normalize_angle(aim_yaw_map - yaw_map)) <= aim_tol_rad_)
      {
        start_visual_align(current, yaw_map);
      }
      return;
    }
    if (distance < arrival_tol_dm_) {
      ++waypoint_index_;
      return;
    }
    const double desired_yaw_map =
      std::atan2(target.y - current.y, target.x - current.x) + origin_yaw_rad_;
    publish_target(target, desired_yaw_map);
  }

  void fail(const std::string & reason)
  {
    // OFF is harmless in mock mode and is mandatory once a real driver is used.
    publish_vision_servo(false, 0.0, 0.0);
    publish_laser("OFF");
    state_ = State::SAFE_STOP;
    if (have_last_valid_pose_) {
      publish_target(last_valid_pose_, last_valid_yaw_map_);
    }
    report("failed:" + reason);
  }

  void reset()
  {
    publish_vision_servo(false, 0.0, 0.0);
    publish_laser("OFF");
    if (have_last_valid_pose_) {
      publish_target(last_valid_pose_, last_valid_yaw_map_);
    }
    path_.clear();
    waypoint_index_ = 0;
    laser_state_.clear();
    pose_failure_active_ = false;
    last_tf_error_.clear();
    state_ = State::IDLE;
    report("ready");
    RCLCPP_INFO(get_logger(), "Mission reset to IDLE via /fire_mission_reset");
  }

  void report(const std::string & status)
  {
    current_status_ = status;
    std_msgs::msg::String message;
    message.data = status;
    status_publisher_->publish(message);
  }

  State state_{State::IDLE};
  static constexpr size_t kInvalidDistrict = std::numeric_limits<size_t>::max();
  double origin_x_m_{-0.25};
  double origin_y_m_{1.35};
  double origin_yaw_rad_{-90.0 * M_PI / 180.0};
  double arrival_tol_dm_{1.2};
  double district_match_tol_dm_{2.0};
  double aim_tol_rad_{8.0 * M_PI / 180.0};
  double laser_on_s_{2.1};
  double grid_dm_{1.0};
  double safety_margin_dm_{1.5};
  double pose_failure_timeout_s_{1.0};
  double tf_max_age_s_{0.5};
  double outside_arena_margin_dm_{3.0};
  int64_t vision_stable_frames_required_{8};
  double vision_error_deadband_x_{0.06};
  double vision_error_deadband_y_{0.06};
  double vision_kp_v_mps_{0.08};
  double vision_kp_w_rps_{0.60};
  double vision_v_max_mps_{0.04};
  double vision_w_max_rps_{0.20};
  double vision_search_w_rps_{0.12};
  double vision_search_yaw_rad_{15.0 * M_PI / 180.0};
  double vision_timeout_s_{12.0};
  double vision_data_timeout_s_{0.5};
  double vision_lost_grace_s_{0.3};
  double vision_max_translation_dm_{2.0};
  double vision_safety_lookahead_dm_{0.5};
  Point home_{};
  Point fire_point_{};
  size_t fire_district_{kInvalidDistrict};
  std::vector<double> obstacles_;
  std::vector<double> district_stop_points_;
  std::vector<Point> path_;
  size_t waypoint_index_{0};
  std::string laser_state_;
  std::string current_status_{"ready"};
  rclcpp::Time laser_started_at_;
  bool pose_failure_active_{false};
  rclcpp::Time pose_failure_started_at_;
  Point last_valid_pose_{};
  double last_valid_yaw_map_{0.0};
  bool have_last_valid_pose_{false};
  std::string last_tf_error_;
  double pose_failure_initial_elapsed_s_{0.0};
  rclcpp::Time vision_started_at_;
  rclcpp::Time last_vision_detection_at_;
  rclcpp::Time last_vision_target_at_;
  Point vision_last_pose_{};
  double vision_entry_yaw_map_{0.0};
  double vision_translation_dm_{0.0};
  double vision_search_direction_{1.0};
  bool vision_detection_seen_{false};
  bool vision_detected_{false};
  bool vision_ever_detected_{false};
  int64_t vision_stable_frames_{0};
  double vision_error_x_{0.0};
  double vision_error_y_{0.0};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
    target_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr laser_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<follower_pkg::msg::VisionServoCommand>::SharedPtr
    vision_servo_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
    fire_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    laser_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_subscription_;
  rclcpp::Subscription<follower_pkg::msg::FireDetection>::SharedPtr
    vision_detection_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FireMissionManager>());
  rclcpp::shutdown();
  return 0;
}
