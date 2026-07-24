#include <gtest/gtest.h>

#include "follower_pkg/laser_handshake.hpp"

using follower_pkg::LaserHandshake;
using follower_pkg::LaserHandshakeResult;

TEST(LaserHandshake, OnRequiresFreshAcknowledgementAndDoesNotExtendOnRetry)
{
  LaserHandshake handshake;
  handshake.begin_on(10.0);

  handshake.note_status("off", 10.01);
  EXPECT_EQ(handshake.evaluate(10.01, 0.5), LaserHandshakeResult::NONE);
  EXPECT_FALSE(handshake.should_retry(10.05, 0.1));
  EXPECT_TRUE(handshake.should_retry(10.10, 0.1));

  handshake.note_status("on", 10.12);
  EXPECT_EQ(
    handshake.evaluate(10.12, 0.5),
    LaserHandshakeResult::ON_CONFIRMED);
  EXPECT_DOUBLE_EQ(handshake.status_at_s(), 10.12);
  EXPECT_FALSE(handshake.should_retry(10.30, 0.1));
}

TEST(LaserHandshake, MissingOnAcknowledgementTimesOut)
{
  LaserHandshake handshake;
  handshake.begin_on(20.0);

  EXPECT_EQ(handshake.evaluate(20.49, 0.5), LaserHandshakeResult::NONE);
  EXPECT_EQ(
    handshake.evaluate(20.50, 0.5),
    LaserHandshakeResult::ACK_TIMEOUT);
}

TEST(LaserHandshake, OffAcknowledgementAndSafetyTimeoutAreDistinct)
{
  LaserHandshake handshake;
  handshake.begin_off(30.0);
  handshake.note_status("on", 30.05);
  EXPECT_EQ(handshake.evaluate(30.05, 0.5), LaserHandshakeResult::NONE);

  handshake.note_status("off", 30.10);
  EXPECT_EQ(
    handshake.evaluate(30.10, 0.5),
    LaserHandshakeResult::OFF_CONFIRMED);

  handshake.begin_off(31.0);
  handshake.note_status("timeout_off", 31.10);
  EXPECT_EQ(
    handshake.evaluate(31.10, 0.5),
    LaserHandshakeResult::SAFETY_TIMEOUT);
}

TEST(LaserHandshake, DriverErrorAndUnexpectedOffFailImmediately)
{
  LaserHandshake handshake;
  handshake.begin_on(40.0);
  handshake.note_status("error", 40.01);
  EXPECT_EQ(
    handshake.evaluate(40.01, 0.5),
    LaserHandshakeResult::DRIVER_ERROR);

  handshake.begin_on(41.0);
  handshake.note_status("on", 41.01);
  ASSERT_EQ(
    handshake.evaluate(41.01, 0.5),
    LaserHandshakeResult::ON_CONFIRMED);
  handshake.note_status("off", 41.10);
  EXPECT_EQ(
    handshake.evaluate(41.10, 0.5),
    LaserHandshakeResult::UNEXPECTED_OFF);

  handshake.begin_on(42.0);
  handshake.note_status("ready", 42.01);
  EXPECT_EQ(handshake.evaluate(42.01, 0.5), LaserHandshakeResult::NONE);
  handshake.note_status("on", 42.02);
  ASSERT_EQ(
    handshake.evaluate(42.02, 0.5),
    LaserHandshakeResult::ON_CONFIRMED);
  handshake.note_status("ready", 42.10);
  EXPECT_EQ(
    handshake.evaluate(42.10, 0.5),
    LaserHandshakeResult::UNEXPECTED_OFF);

  handshake.begin_off(43.0);
  handshake.note_status("ready", 43.01);
  EXPECT_EQ(
    handshake.evaluate(43.01, 0.5),
    LaserHandshakeResult::OFF_CONFIRMED);
}
