// Fire mission state machine. Arena coordinates are dm; TF/control coordinates are m.
#include <algorithm>
#include <cmath>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
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
    declare_parameter<double>("arena_origin_map_x_m", 0.0);
    declare_parameter<double>("arena_origin_map_y_m", 0.0);
    declare_parameter<double>("arena_origin_map_yaw_deg", 0.0);
    declare_parameter<double>("home_x_dm", 13.5);
    declare_parameter<double>("home_y_dm", 2.5);
    declare_parameter<double>("standoff_dm", 4.0);
    declare_parameter<double>("arrival_tol_dm", 1.2);
    declare_parameter<double>("aim_tol_deg", 8.0);
    declare_parameter<double>("laser_on_s", 2.1);
    declare_parameter<double>("grid_dm", 1.0);
    declare_parameter<double>("safety_margin_dm", 1.5);
    declare_parameter<double>("pose_failure_timeout_s", 1.0);
    declare_parameter<double>("tf_max_age_s", 0.5);
    declare_parameter<double>("outside_arena_margin_dm", 3.0);
    declare_parameter<std::vector<double>>(
      "obstacles_dm", std::vector<double>{});

    origin_x_m_ = get_parameter("arena_origin_map_x_m").as_double();
    origin_y_m_ = get_parameter("arena_origin_map_y_m").as_double();
    origin_yaw_rad_ =
      get_parameter("arena_origin_map_yaw_deg").as_double() * M_PI / 180.0;
    home_ = {
      get_parameter("home_x_dm").as_double(),
      get_parameter("home_y_dm").as_double()};
    standoff_dm_ = get_parameter("standoff_dm").as_double();
    arrival_tol_dm_ = get_parameter("arrival_tol_dm").as_double();
    aim_tol_rad_ = get_parameter("aim_tol_deg").as_double() * M_PI / 180.0;
    laser_on_s_ = get_parameter("laser_on_s").as_double();
    grid_dm_ = get_parameter("grid_dm").as_double();
    safety_margin_dm_ = get_parameter("safety_margin_dm").as_double();
    pose_failure_timeout_s_ =
      get_parameter("pose_failure_timeout_s").as_double();
    tf_max_age_s_ = get_parameter("tf_max_age_s").as_double();
    outside_arena_margin_dm_ =
      get_parameter("outside_arena_margin_dm").as_double();
    obstacles_ = get_parameter("obstacles_dm").as_double_array();
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
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {tick();});
    report("ready");
  }

private:
  enum class State {IDLE, WAIT_POSE, DRIVE_FIRE, LASER_ON, DRIVE_HOME, SAFE_STOP};
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
    if (!std::isfinite(standoff_dm_) || standoff_dm_ <= 0.0 ||
      !std::isfinite(arrival_tol_dm_) || arrival_tol_dm_ <= 0.0 ||
      !std::isfinite(aim_tol_rad_) || aim_tol_rad_ <= 0.0 ||
      !std::isfinite(laser_on_s_) || laser_on_s_ <= 0.0 ||
      !std::isfinite(grid_dm_) || grid_dm_ <= 0.0 ||
      !std::isfinite(safety_margin_dm_) || safety_margin_dm_ < 0.0 ||
      !std::isfinite(pose_failure_timeout_s_) || pose_failure_timeout_s_ <= 0.0 ||
      !std::isfinite(tf_max_age_s_) || tf_max_age_s_ <= 0.0 ||
      !std::isfinite(outside_arena_margin_dm_) || outside_arena_margin_dm_ < 0.0)
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

    fire_point_ = requested_fire;
    state_ = State::WAIT_POSE;
    tick();
  }

  void start_mission(const Point & current)
  {
    const std::vector<Point> candidates = {
      {fire_point_.x - standoff_dm_, fire_point_.y},
      {fire_point_.x + standoff_dm_, fire_point_.y},
      {fire_point_.x, fire_point_.y - standoff_dm_},
      {fire_point_.x, fire_point_.y + standoff_dm_}};
    path_.clear();
    const Point planning_start = clamp_to_arena(current);
    for (const auto & candidate : candidates) {
      const auto candidate_path = plan(planning_start, candidate);
      if (!candidate_path.empty() &&
        (path_.empty() || candidate_path.size() < path_.size()))
      {
        path_ = candidate_path;
      }
    }
    if (path_.empty()) {
      fail("unreachable");
      return;
    }
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

  void publish_laser(const char * command)
  {
    std_msgs::msg::String message;
    message.data = command;
    laser_publisher_->publish(message);
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
      tolerate_pose_failure(pose_result, current, yaw_map);
      return;
    }
    accept_pose(current, yaw_map);
    if (state_ == State::WAIT_POSE) {
      start_mission(current);
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
        laser_state_.clear();
        publish_laser("ON");
        laser_started_at_ = now();
        state_ = State::LASER_ON;
        report("extinguishing");
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
        ++waypoint_index_;
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
    publish_laser("OFF");
    state_ = State::SAFE_STOP;
    if (have_last_valid_pose_) {
      publish_target(last_valid_pose_, last_valid_yaw_map_);
    }
    report("failed:" + reason);
  }

  void reset()
  {
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
  double origin_x_m_{0.0};
  double origin_y_m_{0.0};
  double origin_yaw_rad_{0.0};
  double standoff_dm_{4.0};
  double arrival_tol_dm_{1.2};
  double aim_tol_rad_{8.0 * M_PI / 180.0};
  double laser_on_s_{2.1};
  double grid_dm_{1.0};
  double safety_margin_dm_{1.5};
  double pose_failure_timeout_s_{1.0};
  double tf_max_age_s_{0.5};
  double outside_arena_margin_dm_{3.0};
  Point home_{};
  Point fire_point_{};
  std::vector<double> obstacles_;
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
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
    target_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr laser_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
    fire_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    laser_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FireMissionManager>());
  rclcpp::shutdown();
  return 0;
}
