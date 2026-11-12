#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <optional>
#include <thread>
#include <vector>

#include "banyu_msgs/action/navigate_surface_gps.hpp"
#include "banyu_msgs/msg/task4_zones.hpp"
#include "banyu_vehicle_execution/course_fence.hpp"
#include "banyu_vehicle_execution/task4_zones.hpp"
#include "banyu_vehicle_execution/vehicle_execution_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "rx_msgs/msg/course.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using namespace std::chrono_literals;
using namespace banyu_vehicle_execution;

namespace
{
Capsule circle(double x, double y, double r) {return Capsule{{x, y}, {x, y}, r, false};}
}  // namespace

TEST(Task4Geometry, SegmentCrossesCircle)
{
  const auto zone = circle(5.0, 0.0, 2.0);
  EXPECT_TRUE(segmentCrossesZone({0.0, 0.0}, {10.0, 0.0}, zone));
  EXPECT_TRUE(segmentCrossesZone({0.0, 1.5}, {10.0, 1.5}, zone));
  EXPECT_FALSE(segmentCrossesZone({0.0, 2.5}, {10.0, 2.5}, zone));
  // Stops short of the circle, or starts beyond it.
  EXPECT_FALSE(segmentCrossesZone({0.0, 0.0}, {2.5, 0.0}, zone));
  EXPECT_FALSE(segmentCrossesZone({8.0, 0.0}, {12.0, 0.0}, zone));
  // Ends inside.
  EXPECT_TRUE(segmentCrossesZone({0.0, 0.0}, {5.0, 0.0}, zone));
}

TEST(Task4Geometry, DetourGoesAroundAndKeepsBothLegsClear)
{
  const std::vector<Capsule> zones{circle(5.0, 0.2, 2.0)};
  const Vec2 goal{10.0, 0.0};
  const auto aim = detourAim(goal, zones);
  ASSERT_TRUE(aim.has_value());
  EXPECT_GT(std::abs(aim->y), 2.0);
  EXPECT_TRUE(segmentClear({0.0, 0.0}, *aim, zones));
  EXPECT_TRUE(segmentClear(*aim, goal, zones));
  // A clear line is steered straight.
  const auto direct = detourAim({0.0, 10.0}, zones);
  ASSERT_TRUE(direct.has_value());
  EXPECT_DOUBLE_EQ(direct->y, 10.0);
}

TEST(Task4Geometry, DetourAroundMovingCapsuleAvoidsItsWholeSweep)
{
  // The object sweeps from (5,-6) to (5,6): the way east has to go around an end of it.
  const std::vector<Capsule> zones{Capsule{{5.0, -6.0}, {5.0, 6.0}, 2.0, true}};
  const auto aim = detourAim({10.0, 0.0}, zones);
  ASSERT_TRUE(aim.has_value());
  // The way round a long sweep takes two legs: the first waypoint is clear and past an end of the sweep.
  EXPECT_TRUE(segmentClear({0.0, 0.0}, *aim, zones));
  EXPECT_FALSE(insideAny(*aim, zones));
  EXPECT_GT(std::abs(aim->y), 7.0);
  // Following the waypoints reaches the goal without entering the zone.
  Vec2 pos = *aim;
  for (int i = 0; i < 4 && length(pos - Vec2{10.0, 0.0}) > 1e-6; ++i) {
    const auto next = detourAim(Vec2{10.0, 0.0} - pos, zones.empty() ? zones : std::vector<Capsule>{
        Capsule{zones[0].a - pos, zones[0].b - pos, zones[0].radius, true}});
    ASSERT_TRUE(next.has_value());
    EXPECT_FALSE(insideAny(pos + *next, zones));
    pos = pos + *next;
  }
  EXPECT_NEAR(pos.x, 10.0, 1e-6);
}

TEST(Task4Geometry, ExitVectorLeavesByTheShortestWay)
{
  EXPECT_FALSE(exitVector({10.0, 0.0}, {circle(0.0, 0.0, 3.0)}, 0.5).has_value());
  const auto exit = exitVector({1.0, 0.0}, {circle(0.0, 0.0, 3.0)}, 0.5);
  ASSERT_TRUE(exit.has_value());
  EXPECT_NEAR(exit->direction.x, 1.0, 1e-9);
  EXPECT_NEAR(exit->distance_m, 2.0 + 0.5, 1e-9);
  // On the centre it still picks a direction; beside a moving object's sweep it leaves sideways to the travel.
  EXPECT_TRUE(exitVector({0.0, 0.0}, {circle(0.0, 0.0, 3.0)}, 0.5).has_value());
  const auto side = exitVector({1.0, 2.0}, {Capsule{{0.0, -10.0}, {0.0, 10.0}, 3.0, true}}, 0.5);
  ASSERT_TRUE(side.has_value());
  EXPECT_NEAR(side->direction.x, 1.0, 1e-9);
  EXPECT_NEAR(side->direction.y, 0.0, 1e-9);
}

TEST(Task4Geometry, DivePointShiftsAlongTheBuoyLineThenAnywhere)
{
  const Vec2 start{-10.0, 0.0};
  const Vec2 end{10.0, 0.0};
  const std::vector<Vec2> buoys{start, end};
  const auto line = std::make_optional(std::make_pair(start, end));
  // Outside: unchanged.
  const auto same = shiftDivePoint({0.0, 0.0}, {circle(0.0, 8.0, 3.0)}, buoys, 1.5, line);
  ASSERT_TRUE(same.has_value());
  EXPECT_FALSE(same->shifted);
  // Zone on the dive point: slides along the line to the nearer edge.
  const std::vector<Capsule> zones{circle(1.0, 0.0, 3.0)};
  const auto shifted = shiftDivePoint({0.0, 0.0}, zones, buoys, 1.5, line);
  ASSERT_TRUE(shifted.has_value());
  EXPECT_TRUE(shifted->shifted);
  EXPECT_TRUE(shifted->along_line);
  EXPECT_NEAR(shifted->point.y, 0.0, 1e-9);
  EXPECT_NEAR(shifted->point.x, -2.0 - 0.1, 0.01);
  EXPECT_FALSE(insideAny(shifted->point, zones));
  // A zone that reaches past both buoys: the line has no point outside it clear of the buoys, so the nearest point off the line.
  const std::vector<Capsule> big{circle(0.0, 0.0, 11.0)};
  const auto off = shiftDivePoint({0.0, 0.0}, big, buoys, 1.5, line);
  ASSERT_TRUE(off.has_value());
  EXPECT_TRUE(off->shifted);
  EXPECT_FALSE(off->along_line);
  EXPECT_FALSE(insideAny(off->point, big));
  EXPECT_GE(length(off->point - start), 1.5);
  EXPECT_GE(length(off->point - end), 1.5);
}

TEST(Task4Geometry, MovingObjectIsProjectedFromHeadingSpeedAndAge)
{
  MovingObject object;
  object.heading_enu_rad = M_PI / 2.0;  // north
  object.speed_mps = 2.0;
  object.radius_m = 5.0;
  const auto p = projectMoving(object, 10.0);
  EXPECT_NEAR(p.x, 0.0, 1e-9);
  EXPECT_NEAR(p.y, 20.0, 1e-9);

  ZoneField field;
  field.active = true;
  field.moving = object;
  field.margin_m = 1.0;
  field.horizon_s = 30.0;
  // Alert position is the reference; 5 s later it is 10 m north, and the sweep reaches 60 m beyond that.
  const auto zones = zonesAround(field, 0.0, 0.0, 5.0);
  ASSERT_EQ(zones.size(), 1u);
  EXPECT_TRUE(zones[0].moving);
  EXPECT_NEAR(zones[0].a.y, 10.0, 1e-6);
  EXPECT_NEAR(zones[0].b.y, 10.0 + 60.0, 1e-6);
  EXPECT_NEAR(zones[0].radius, 6.0, 1e-9);
  // A point ahead of the object is inside the projected zone, the same point behind it is not.
  EXPECT_TRUE(insideAny({0.0, 50.0}, zones));
  EXPECT_FALSE(insideAny({0.0, -10.0}, zones));
  // Not active until a zone message arrived.
  field.active = false;
  EXPECT_TRUE(zonesAround(field, 0.0, 0.0).empty());
}

TEST(Task4Geometry, TruncateStopsAtTheZoneEdge)
{
  const auto zones = std::vector<Capsule>{circle(10.0, 0.0, 3.0)};
  const auto stop = truncateAtZone({10.0, 0.0}, zones, 0.5);
  ASSERT_TRUE(stop.has_value());
  EXPECT_NEAR(stop->x, 7.0 - 0.5, 0.15);
  EXPECT_FALSE(truncateAtZone({0.0, 10.0}, zones, 0.5).has_value());
}

TEST(CourseFenceGeometry, DistanceIsSignedAndPointsIntoTheCourse)
{
  // Clockwise 20 m square: the winding must not matter.
  const std::vector<Vec2> square{{-20.0, -20.0}, {-20.0, 20.0}, {20.0, 20.0}, {20.0, -20.0}};
  const auto inside = fenceDistance({18.0, 0.0}, square);
  EXPECT_NEAR(inside.clearance_m, 2.0, 1e-9);
  EXPECT_NEAR(inside.inward.x, -1.0, 1e-9);
  const auto outside = fenceDistance({25.0, 0.0}, square);
  EXPECT_NEAR(outside.clearance_m, -5.0, 1e-9);
  EXPECT_NEAR(outside.inward.x, -1.0, 1e-9);
  // A goal past a corner comes back inside by the margin on both sides.
  const Vec2 pulled = pullInside({30.0, 30.0}, square, 3.0);
  EXPECT_GE(fenceDistance(pulled, square).clearance_m, 3.0 - 1e-3);
  EXPECT_LT(pulled.x, 17.1);
  EXPECT_LT(pulled.y, 17.1);
}

TEST(CourseFenceGeometry, VelocityBrakesOutwardAndPushesBackIn)
{
  CourseFence fence;
  fence.margin_m = 3.0;
  fence.brake_m = 3.0;
  fence.return_speed_mps = 0.4;
  const FenceDistance far{10.0, {-1.0, 0.0}};
  EXPECT_NEAR(fenceVelocity({1.0, 0.5}, far, fence).x, 1.0, 1e-9);
  // Halfway through the brake band: half the outward speed, the sideways part kept.
  const auto braking = fenceVelocity({1.0, 0.5}, FenceDistance{4.5, {-1.0, 0.0}}, fence);
  EXPECT_NEAR(braking.x, 0.5, 1e-9);
  EXPECT_NEAR(braking.y, 0.5, 1e-9);
  // Inside the margin: pushed back in whatever was asked, sideways part kept.
  const auto pushed = fenceVelocity({1.0, 0.5}, FenceDistance{1.0, {-1.0, 0.0}}, fence);
  EXPECT_LT(pushed.x, -0.19);
  EXPECT_NEAR(pushed.y, 0.5, 1e-9);
  // Inward commands are never slowed.
  EXPECT_NEAR(fenceVelocity({-0.8, 0.0}, FenceDistance{1.0, {-1.0, 0.0}}, fence).x, -0.8, 1e-9);
}

class Task4NavigationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  }

  void SetUp() override
  {
    node_ = std::make_shared<VehicleExecutionNode>(rclcpp::NodeOptions());
    gps_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("/mavros/global_position/raw/fix", rclcpp::SensorDataQoS());
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose", rclcpp::SensorDataQoS());
    zones_pub_ = node_->create_publisher<banyu_msgs::msg::Task4Zones>(
      "/banyu/task4/zones", rclcpp::QoS(1).reliable().transient_local());
    course_pub_ = node_->create_publisher<rx_msgs::msg::Course>("/system/mission/course", rclcpp::QoS(10).reliable());
    preview_sub_ = node_->create_subscription<mavros_msgs::msg::PositionTarget>(
      "/banyu/execution/preview_target", 10,
      [this](mavros_msgs::msg::PositionTarget::SharedPtr m) {last_target_ = *m;});
    client_ = rclcpp_action::create_client<NavAction>(node_, "/banyu/actions/navigate_surface_gps");
    ASSERT_TRUE(client_->wait_for_action_server(2s));
  }

  // Degrees for an offset from (0, 0) at the equator.
  static double degNorth(double m) {return m / 6378137.0 * 180.0 / M_PI;}
  static double degEast(double m) {return m / 6378137.0 * 180.0 / M_PI;}

  void feed(double east, double north)
  {
    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = node_->now();
    fix.latitude = degNorth(north);
    fix.longitude = degEast(east);
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    gps_pub_->publish(fix);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = node_->now();
    pose.header.frame_id = "map";
    pose.pose.orientation.w = 1.0;
    pose_pub_->publish(pose);
  }

  void spin(double east, double north, int cycles)
  {
    for (int i = 0; i < cycles; ++i) {
      feed(east, north);
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(10ms);
    }
  }

  void publishStaticZone(double east, double north, double radius)
  {
    banyu_msgs::msg::Task4Zones zones;
    zones.static_latitude_deg = {degNorth(north)};
    zones.static_longitude_deg = {degEast(east)};
    zones.static_radius_m = {radius};
    zones.last_event = "keep_out_zone";
    zones_pub_->publish(zones);
  }

  // A moving object alerted `age_s` ago at (0, 0), heading north at 1 m/s, radius 1 m.
  void publishMovingObject(double age_s)
  {
    banyu_msgs::msg::Task4Zones zones;
    zones.moving_active = true;
    zones.moving_heading_enu_rad = M_PI / 2.0;
    zones.moving_speed_mps = 1.0;
    zones.moving_radius_m = 1.0;
    zones.moving_stamp = node_->now() - rclcpp::Duration::from_seconds(age_s);
    zones.last_event = "moving_object";
    zones_pub_->publish(zones);
  }

  // Square course with its east edge `east_m` east of (0, 0).
  void publishCourse(double east_m)
  {
    rx_msgs::msg::Course course;
    course.course_id = "TEST";
    for (const auto & [e, n] : std::vector<std::pair<double, double>>{
        {-100.0, -100.0}, {east_m, -100.0}, {east_m, 100.0}, {-100.0, 100.0}, {-100.0, -100.0}})
    {
      rx_msgs::msg::LatLng p;
      p.latitude = degNorth(n);
      p.longitude = degEast(e);
      course.boundary.push_back(p);
    }
    course_pub_->publish(course);
  }

  bool sendGoal(double east_m)
  {
    NavAction::Goal goal;
    goal.target_latitude_deg = 0.0;
    goal.target_longitude_deg = degEast(east_m);
    goal.tolerance_m = 1.0;
    auto future = client_->async_send_goal(goal);
    const auto until = std::chrono::steady_clock::now() + 2s;
    while (future.wait_for(0ms) != std::future_status::ready && std::chrono::steady_clock::now() < until) {
      feed(0.0, 0.0);
      rclcpp::spin_some(node_);
    }
    return future.wait_for(0ms) == std::future_status::ready && future.get() != nullptr;
  }

  using NavAction = banyu_msgs::action::NavigateSurfaceGps;
  std::shared_ptr<VehicleExecutionNode> node_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<banyu_msgs::msg::Task4Zones>::SharedPtr zones_pub_;
  rclcpp::Publisher<rx_msgs::msg::Course>::SharedPtr course_pub_;
  rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr preview_sub_;
  rclcpp_action::Client<NavAction>::SharedPtr client_;
  mavros_msgs::msg::PositionTarget last_target_;
};

TEST_F(Task4NavigationTest, SteersAroundAZoneOnTheTransitLine)
{
  ASSERT_TRUE(sendGoal(30.0));
  spin(0.0, 0.0, 40);
  EXPECT_GT(last_target_.velocity.x, 0.3);
  EXPECT_NEAR(last_target_.velocity.y, 0.0, 0.05);

  // A zone across the line (radius 2 + margin 2): the vehicle swings sideways instead of straight at it.
  publishStaticZone(15.0, 0.5, 2.0);
  spin(0.0, 0.0, 40);
  EXPECT_GT(std::abs(last_target_.velocity.y), 0.15);
  EXPECT_GT(last_target_.velocity.x, 0.0);
}

TEST_F(Task4NavigationTest, LeavesAZonePublishedOnTopOfIt)
{
  ASSERT_TRUE(sendGoal(30.0));
  // The zone is centred 1 m east of the vehicle: leaving means going west, away from the goal.
  publishStaticZone(1.0, 0.0, 2.0);
  spin(0.0, 0.0, 40);
  EXPECT_LT(last_target_.velocity.x, -0.2);
}

TEST_F(Task4NavigationTest, IdleVehicleInsideAZoneMovesOut)
{
  // No goal at all (standby): the idle guard still pushes the vehicle out.
  publishStaticZone(0.0, 1.0, 2.0);
  spin(0.0, 0.0, 40);
  EXPECT_LT(last_target_.velocity.y, -0.2);
  // Outside the zone it stays put.
  spin(0.0, -20.0, 20);
  EXPECT_NEAR(last_target_.velocity.y, 0.0, 1e-6);
}

TEST_F(Task4NavigationTest, OldMovingObjectAlertKeepsItsAge)
{
  // A fresh alert sweeps over the vehicle: the idle guard moves it out.
  publishMovingObject(0.0);
  spin(0.0, 0.0, 40);
  EXPECT_GT(std::hypot(last_target_.velocity.x, last_target_.velocity.y), 0.2);
  // 90 s later the object is 90 m north; it must not reappear at its alert position.
  publishMovingObject(90.0);
  spin(0.0, 0.0, 40);
  EXPECT_NEAR(std::hypot(last_target_.velocity.x, last_target_.velocity.y), 0.0, 1e-6);
}

TEST_F(Task4NavigationTest, SteersAroundAUavBuoyButNotTheOneAtTheGoal)
{
  auto obstacles_pub = node_->create_publisher<banyu_msgs::msg::SurfaceObstacles>(
    "/banyu/surface/obstacles", rclcpp::QoS(1).reliable().transient_local());
  ASSERT_TRUE(sendGoal(30.0));

  // A UAV-mapped buoy on the goal itself (it is the buoy driven to) changes nothing.
  banyu_msgs::msg::SurfaceObstacles at_goal;
  at_goal.source = "uav_safe_passage";
  at_goal.latitude_deg = {0.0};
  at_goal.longitude_deg = {degEast(30.0)};
  at_goal.radius_m = {2.0};
  obstacles_pub->publish(at_goal);
  spin(0.0, 0.0, 40);
  EXPECT_GT(last_target_.velocity.x, 0.3);
  EXPECT_NEAR(last_target_.velocity.y, 0.0, 0.05);

  // One just north of the line replaces that list: the vehicle swings south, the short way round it.
  banyu_msgs::msg::SurfaceObstacles on_line = at_goal;
  on_line.latitude_deg = {degNorth(0.5)};
  on_line.longitude_deg = {degEast(15.0)};
  obstacles_pub->publish(on_line);
  spin(0.0, 0.0, 40);
  EXPECT_LT(last_target_.velocity.y, -0.08);
  EXPECT_GT(last_target_.velocity.x, 0.0);
}

TEST(Task4Geometry, DetourReportsThePathLength)
{
  const std::vector<Capsule> zones{circle(5.0, 0.0, 2.0)};
  double length_m = 0.0;
  ASSERT_TRUE(detourAim({0.0, 10.0}, zones, {}, 0.0, &length_m).has_value());
  EXPECT_DOUBLE_EQ(length_m, 10.0);
  ASSERT_TRUE(detourAim({10.0, 0.0}, zones, {}, 0.0, &length_m).has_value());
  EXPECT_GT(length_m, 10.0);
  EXPECT_LT(length_m, 14.0);
}

TEST(Task4Geometry, KeepDetourPointUntilReachedBlockedOrClearlyBeaten)
{
  const std::vector<Capsule> zones{circle(5.0, 0.0, 2.0)};
  const Vec2 goal{10.0, 0.0};
  const Vec2 left{5.0, 2.6};
  // The other side is no shorter: keep it.
  EXPECT_TRUE(keepDetourPoint(left, goal, zones, 10.6));
  // A fresh plan shorter by more than the switch gain wins.
  EXPECT_FALSE(keepDetourPoint(left, goal, zones, 7.0));
  // No fresh plan (boxed in): keep the point while it is still valid.
  EXPECT_TRUE(keepDetourPoint(left, goal, zones, std::nullopt));
  // Reached, inside an obstacle, or no longer in clear view.
  EXPECT_FALSE(keepDetourPoint({0.3, 0.1}, goal, zones, std::nullopt));
  EXPECT_FALSE(keepDetourPoint({5.0, 1.0}, goal, zones, std::nullopt));
  EXPECT_FALSE(keepDetourPoint({10.0, 0.5}, goal, zones, std::nullopt));
}

// The UAV's Task 1 buoy field of the 2026-11-12 trial (ENU m), 1.2 m keep-out plus the 0.75 m surface obstacle margin.
// Crossing it north/south and east/west with the position jittering by the GPS noise measured on the water
// (+-0.2 m): every crossing arrives, never comes within 1.5 m of a buoy centre and does not dither (path at most 1.5
// times the straight line).
TEST(Task4Geometry, TrialBuoyFieldCrossingsStayClearWithDetourHysteresis)
{
  const std::vector<Vec2> field{{-0.2, 0.0}, {3.4, 9.0}, {7.1, 2.2}, {7.1, 5.3}, {0.2, 8.6}, {7.1, 8.7}};
  const double radius = 1.2 + 0.75;
  for (int dir = 0; dir < 4; ++dir) {
    for (double off = -3.0; off <= 3.0; off += 1.5) {
      const double c = off + 3.5;
      Vec2 pos = dir == 0 ? Vec2{c, -9.0} : dir == 1 ? Vec2{c, 18.0} : dir == 2 ? Vec2{-10.0, c + 1.0} : Vec2{17.0, c + 1.0};
      const Vec2 goal = dir == 0 ? Vec2{c, 18.0} : dir == 1 ? Vec2{c, -9.0} : dir == 2 ? Vec2{17.0, c + 1.0} :
        Vec2{-10.0, c + 1.0};
      std::optional<Vec2> committed;
      double min_clear = 1e9;
      double travelled = 0.0;
      const Vec2 start = pos;
      bool arrived = false;
      for (int step = 0; step < 600 && !arrived; ++step) {
        const Vec2 measured = pos + Vec2{0.2 * std::sin(step * 0.7), 0.2 * std::cos(step * 1.3)};
        std::vector<Capsule> discs;
        for (const auto & b : field) {discs.push_back(circle(b.x - measured.x, b.y - measured.y, radius));}
        const Vec2 target = goal - measured;
        // As the server: inside a disc (the jitter carried the fix across its edge), leave it first.
        if (const auto exit = exitVector(Vec2{}, discs, 0.5)) {
          pos = pos + exit->direction * 0.2;
          travelled += 0.2;
          continue;
        }
        double fresh = 0.0;
        auto aim = detourAim(target, discs, {}, 0.0, &fresh);
        if (committed && keepDetourPoint(*committed - measured, target, discs, aim ? std::optional<double>(fresh) :
          std::nullopt))
        {
          aim = *committed - measured;
        } else {
          committed.reset();
        }
        if (aim && !committed && length(*aim - target) > 1e-6) {
          committed = measured + *aim;
        }
        // Boxed in: hold, as the server does.
        const double len = aim ? length(*aim) : 0.0;
        if (len > 1e-6) {
          pos = pos + *aim * (std::min(0.2, len) / len);
          travelled += std::min(0.2, len);
        }
        for (const auto & b : field) {min_clear = std::min(min_clear, length(pos - b));}
        arrived = length(goal - pos) < 0.4;
      }
      EXPECT_TRUE(arrived) << "direction " << dir << " offset " << off;
      EXPECT_GE(min_clear, 1.5) << "direction " << dir << " offset " << off;
      EXPECT_LE(travelled, 1.5 * length(goal - start)) << "direction " << dir << " offset " << off;
    }
  }
}

TEST_F(Task4NavigationTest, CourseFenceKeepsTheTransitInside)
{
  // Goal 30 m east, the boundary 20 m east: the goal is pulled 3 m (margin) inside the boundary.
  publishCourse(20.0);
  ASSERT_TRUE(sendGoal(30.0));
  spin(0.0, 0.0, 40);
  EXPECT_GT(last_target_.velocity.x, 0.3);
  // Inside the margin (1 m from the boundary): driven back in, never further out.
  spin(19.0, 0.0, 40);
  EXPECT_LT(last_target_.velocity.x, -0.1);
  // Outside the boundary: driven back in too.
  spin(22.0, 0.0, 40);
  EXPECT_LT(last_target_.velocity.x, -0.1);
}

TEST_F(Task4NavigationTest, IdleVehicleOutsideTheCourseFenceMovesBackIn)
{
  publishCourse(20.0);
  spin(19.0, 0.0, 40);
  EXPECT_LT(last_target_.velocity.x, -0.1);
  // Well inside it stays put.
  spin(0.0, 0.0, 20);
  EXPECT_NEAR(last_target_.velocity.x, 0.0, 1e-6);
}

TEST_F(Task4NavigationTest, GoalFarOutsideTheCourseFenceIsRefused)
{
  publishCourse(20.0);
  spin(0.0, 0.0, 10);
  ASSERT_TRUE(sendGoal(100.0));
  spin(0.0, 0.0, 40);
  EXPECT_NEAR(last_target_.velocity.x, 0.0, 1e-6);
}
