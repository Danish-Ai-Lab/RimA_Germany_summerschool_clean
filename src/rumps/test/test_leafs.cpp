#include "../include/rumps/rmp.hpp"
#include "../include/rumps/rmp_leaf.hpp"
#include "../include/rumps/rmp_mapping.hpp"
#include "../include/rumps/spaces.hpp"
#include "rumps/containers.hpp"
#include <gtest/gtest.h>

using namespace rumps;

TEST(RMPLeafTest, SmokeTest) {
  Eigen::Vector3d c(1.0, 0.0, 0.0);
  Root<SE3> root;
  auto &position_map = root.addChild<Position3DMap>();
  position_map.addChild<PositionRepulsor<Position3D>>(c);

  auto x = Position3DMap::DomainVec::Zero();
  auto x_dot = Position3DMap::DomainVec::Zero();

  root.solve(x, x_dot);
}

TEST(RMPLeafStaticContainerTest, SmokeTest) {
  Eigen::Vector3d c(1.0, 0.0, 0.0);
  Root<SE3, StaticNodeContainer<SE3, Position3DMap, 16>> root;
  root.addChild<Position3DMap>();

  auto x = Position3DMap::DomainVec::Zero();
  auto x_dot = Position3DMap::DomainVec::Zero();

  root.solve(x, x_dot);
}

TEST(RMPLeafTest, PositionRepulsorTest) {
  Eigen::Vector3d c(1.0, 0.0, 0.0);
  Root<SE3> root;
  auto &position_map = root.addChild<Position3DMap>();
  position_map.addChild<PositionRepulsor<Position3D>>(c);

  auto x = Position3DMap::DomainVec::Zero();
  auto x_dot = Position3DMap::DomainVec::Zero();

  Position3DMap::DomainVec a = root.solve(x, x_dot);

  // expect it to push with some acceleration away from the obstacle
  EXPECT_EQ(a.size(), x.size());
  EXPECT_LT(a[0], 0.0);
  for (int i = 1; i < a.size(); ++i) {
    EXPECT_NEAR(a[i], 0.0, 1e-6);
  }
}

TEST(RMPNavpiTest, PositionRepulsorCircleDirectionTest) {
  Eigen::Vector2d c(0.0, 0.0);
  double alpha = 1e-5;

  Root<SE3> root;
  auto &position_map = root.addChild<Position2DMap>();
  position_map.addChild<PositionRepulsor<Position2D>>(c);

  double start_radius = 0.1;
  double end_radius = 2.0;
  double radius_increment = 0.1;

  for (double radius = start_radius; radius < end_radius;
       radius += radius_increment) {

    double angle_increment = 0.1;

    for (double theta = 0.0; theta < 2 * M_PI; theta += angle_increment) {
      double xi = cos(theta) * radius;
      double yi = sin(theta) * radius;
      Root<SE3>::DomainVec x(xi, yi, 0, 0, 0, 0);
      auto x_dot = Root<SE3>::DomainVec::Zero();
      Eigen::Vector3d origin_direction = x.head<3>().normalized();

      Root<SE3>::DomainVec a = root.solve(x, x_dot);
      Eigen::Vector3d push_direction = a.head<3>().normalized();
      EXPECT_EQ(a.size(), x.size());

      // expect it to push in towards the position (so away from the obstacle)
      EXPECT_NEAR(push_direction.dot(origin_direction), 1.0, 1e-6)
          << "Push direction is not opposite to obstacle at angle " << theta
          << '\n'
          << "x: " << x.x() << ", " << x.y() << ", " << x.z() << '\n'
          << "origin direction: " << origin_direction.x() << ", "
          << origin_direction.y() << ", " << origin_direction.z() << '\n'
          << "push direction: " << push_direction.x() << ", "
          << push_direction.y() << ", " << push_direction.z() << '\n'
          << "alpha: " << alpha << '\n'
          << "radius: " << radius << '\n';
    }
  }
}
TEST(RMPLeafTest, DecreasingRepulsionTest) {
  Eigen::Vector3d c(1.0, 0.0, 0.0);
  Root<SE3> root;
  auto &position_map = root.addChild<Position3DMap>();
  position_map.addChild<PositionRepulsor<Position3D>>(c);

  size_t steps = 10;
  double step_size = 1.8 / steps;
  double start_distance = -1.0;

  std::vector<Position3DMap::DomainVec> accelerations;

  // generate trajectory
  Position3DMap::DomainVec x = Position3DMap::DomainVec::Zero();
  for (size_t i = 0; i < steps; ++i) {
    x(0) = start_distance + i * step_size;
    auto x_dot = Position3DMap::DomainVec::Zero();

    Position3DMap::DomainVec a = root.solve(x, x_dot);
    accelerations.push_back(a);
  }

  // expect it to push harder the closer we are (they are negative, pushing
  // away)
  for (size_t i = 1; i < accelerations.size(); ++i) {
    EXPECT_LT(accelerations[i][0], 0.0)
        << "Acceleration was supposed to be negative, pushing away from the "
           "obstacle";
    EXPECT_GT(std::abs(accelerations[i][0]), std::abs(accelerations[i - 1][0]))
        << "Total acceleration did not increase as expected at step " << i;
  }
}

TEST(RMPLeafTest, PositionAttractorTest) {
  Eigen::Vector3d c(1.0, 0.0, 0.0);
  Root<SE3> root;
  auto &position_map = root.addChild<Position3DMap>();
  position_map.addChild<PositionAttractor<Position3D>>(c);

  auto x = Position3DMap::DomainVec::Zero();
  auto x_dot = Position3DMap::DomainVec::Zero();

  Position3DMap::DomainVec a = root.solve(x, x_dot);

  // expect it to pull with some acceleration towards the goal
  EXPECT_GT(a[0], 0.0);
  for (int i = 1; i < a.size(); ++i) {
    EXPECT_NEAR(a[i], 0.0, 1e-6);
  }
}

TEST(RMPLeafTest, IncreasingAttractorTest) {
  Eigen::Vector3d c(1.0, 0.0, 0.0);
  Root<SE3> root;
  auto &position_map = root.addChild<Position3DMap>();
  position_map.addChild<PositionAttractor<Position3D>>(c);

  size_t steps = 10;
  double step_size = 1.8 / steps;
  double start_distance = -1.0;

  std::vector<Position3DMap::DomainVec> accelerations;

  // generate trajectory
  Position3DMap::DomainVec x = Position3DMap::DomainVec::Zero();
  for (size_t i = 0; i < steps; ++i) {
    x(0) = start_distance + i * step_size;
    auto x_dot = Position3DMap::DomainVec::Zero();

    Position3DMap::DomainVec a = root.solve(x, x_dot);
    accelerations.push_back(a);
  }

  // expect it to pull less the closer we are
  for (size_t i = 1; i < accelerations.size(); ++i) {
    EXPECT_GT(accelerations[i][0], 0.0)
        << "Acceleration was supposed to be positive, pulling towards the goal";
    EXPECT_LT(std::abs(accelerations[i][0]), std::abs(accelerations[i - 1][0]))
        << "Total acceleration did not decrease as expected at step " << i;
  }
}
