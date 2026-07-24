#ifndef FOLLOWER_PKG__LASER_HANDSHAKE_HPP_
#define FOLLOWER_PKG__LASER_HANDSHAKE_HPP_

#include <string>

namespace follower_pkg
{

enum class LaserHandshakePhase
{
  IDLE,
  WAIT_ON,
  ON,
  WAIT_OFF,
};

enum class LaserHandshakeResult
{
  NONE,
  ON_CONFIRMED,
  OFF_CONFIRMED,
  DRIVER_ERROR,
  SAFETY_TIMEOUT,
  UNEXPECTED_OFF,
  ACK_TIMEOUT,
};

class LaserHandshake
{
public:
  void begin_on(double now_s)
  {
    begin(LaserHandshakePhase::WAIT_ON, now_s);
  }

  void begin_off(double now_s)
  {
    begin(LaserHandshakePhase::WAIT_OFF, now_s);
  }

  void note_status(const std::string & status, double now_s)
  {
    status_ = status;
    status_at_s_ = now_s;
  }

  LaserHandshakeResult evaluate(double now_s, double ack_timeout_s)
  {
    if (status_ == "error") {
      phase_ = LaserHandshakePhase::IDLE;
      return LaserHandshakeResult::DRIVER_ERROR;
    }
    if (status_ == "timeout_off") {
      phase_ = LaserHandshakePhase::IDLE;
      return LaserHandshakeResult::SAFETY_TIMEOUT;
    }
    if (phase_ == LaserHandshakePhase::WAIT_ON && status_ == "on") {
      phase_ = LaserHandshakePhase::ON;
      return LaserHandshakeResult::ON_CONFIRMED;
    }
    if (phase_ == LaserHandshakePhase::ON &&
      (status_ == "off" || status_ == "ready"))
    {
      phase_ = LaserHandshakePhase::IDLE;
      return LaserHandshakeResult::UNEXPECTED_OFF;
    }
    if (phase_ == LaserHandshakePhase::WAIT_OFF &&
      (status_ == "off" || status_ == "ready"))
    {
      phase_ = LaserHandshakePhase::IDLE;
      return LaserHandshakeResult::OFF_CONFIRMED;
    }
    if (waiting() && now_s - started_at_s_ >= ack_timeout_s) {
      phase_ = LaserHandshakePhase::IDLE;
      return LaserHandshakeResult::ACK_TIMEOUT;
    }
    return LaserHandshakeResult::NONE;
  }

  bool should_retry(double now_s, double retry_interval_s)
  {
    constexpr double kTimeEpsilonS = 1e-9;
    if (!waiting() ||
      now_s - last_command_at_s_ + kTimeEpsilonS < retry_interval_s)
    {
      return false;
    }
    last_command_at_s_ = now_s;
    return true;
  }

  bool waiting() const
  {
    return phase_ == LaserHandshakePhase::WAIT_ON ||
           phase_ == LaserHandshakePhase::WAIT_OFF;
  }

  bool waiting_for_on() const
  {
    return phase_ == LaserHandshakePhase::WAIT_ON;
  }

  double status_at_s() const
  {
    return status_at_s_;
  }

private:
  void begin(LaserHandshakePhase phase, double now_s)
  {
    phase_ = phase;
    started_at_s_ = now_s;
    last_command_at_s_ = now_s;
    status_.clear();
    status_at_s_ = now_s;
  }

  LaserHandshakePhase phase_{LaserHandshakePhase::IDLE};
  std::string status_;
  double started_at_s_{0.0};
  double last_command_at_s_{0.0};
  double status_at_s_{0.0};
};

}  // namespace follower_pkg

#endif  // FOLLOWER_PKG__LASER_HANDSHAKE_HPP_
