#include "../include/rumps/rumps.hpp"
#include <Eigen/Eigen>
#include <gtest/gtest.h>

using namespace rumps;

template <typename M> double getCondition(M A) {
  Eigen::Matrix2d A2d = A.template block<2, 2>(0, 0);
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(A2d);
  double cond = svd.singularValues()(0) /
                svd.singularValues()(svd.singularValues().size() - 1);

  return cond;
}

// close to the goal and far away from the obstalce the acceleartion should
// point roughly in the direction of the goal
TEST(RMPLeafTest, RepulsorAttractorGoalProximityDirectionTest) {
  Root<SE3> root;
  auto& position_map = root.addChild<Position2DMap>();

  Eigen::Vector2d c(0.0, 0.0);
  auto& repulsor = position_map.addChild<PositionRepulsor<Position2D>>(c);

  Eigen::Vector2d goal(3.0, 0.0);
  auto& attractor = position_map.addChild<PositionAttractor<Position2D>>(goal);

  root.addChild<TikhonovRegularizer<SE3>>(1e-2);

  double start_radius = 0.1;
  double end_radius = 1.0;
  double radius_increment = 0.1;

  for (double radius = start_radius; radius < end_radius;
       radius += radius_increment) {

    double angle_increment = 0.1;

    for (double theta = 0.0; theta < (2 * M_PI); theta += angle_increment) {

      double xi_goal = goal(0) + cos(theta) * radius;
      double yi_goal = goal(1) + sin(theta) * radius;
      Root<SE3>::DomainVec x(xi_goal, yi_goal, 0, 0, 0, 0);
      auto x_dot = Root<SE3>::DomainVec::Zero();

      Eigen::Vector3d goal_direction =
          (Eigen::Vector3d(goal.x(), goal.y(), 0.0) - x.head<3>()).normalized();
      Eigen::Vector3d obstacle_direction =
          (Eigen::Vector3d(c.x(), c.y(), 0.0) - x.head<3>()).normalized();

      Root<SE3>::DomainVec a = root.solve(x, x_dot);
      Eigen::Vector3d push_direction = a.head<3>().normalized();

      Root<SE3>::DomainVec root_f;
      Root<SE3>::MatrixM root_M;
      root.pullback(root_f, root_M);

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(attractor.M());

      // expect it to push in the direction of the goal
      EXPECT_GT(push_direction.dot(goal_direction), 0.7)
          << "Push direction does not point in the direction of the goal at "
             "angle "
          << theta << '\n'
          << "x: " << x.x() << ", " << x.y() << ", " << x.z() << '\n'
          << "a: " << a.x() << ", " << a.y() << ", " << a.z() << '\n'
          << "goal direction: " << goal_direction.x() << ", "
          << goal_direction.y() << ", " << goal_direction.z() << '\n'
          << "push direction: " << push_direction.x() << ", "
          << push_direction.y() << ", " << push_direction.z() << '\n'
          << "root f: " << root_f.x() << ", " << root_f.y() << ", "
          << root_f.z() << '\n'
          << "angle between force and acceleration: " << acos(root_f.normalized().dot(a.normalized())) * 180.0 / M_PI << "\n"
          << "root M: " << root_M << '\n'
          << "position map M: " << position_map.M() << '\n'
          << "position map J: " << position_map.J_() << '\n'
          << "position map M condition number: " << getCondition(position_map.M()) << "\n"
          << "repulsor M: " << repulsor.M() << '\n'
          << "repulsor J: " << repulsor.J_() << '\n'
          << "repulsor M condition number: " << getCondition(repulsor.M()) << "\n"
          << "attractor M: " << attractor.M() << '\n'
          << "attractor J: " << attractor.J_() << '\n'
          << "attractor M condition number: " << getCondition(attractor.M()) << "\n"
          << "Eigenvalues:\n" << solver.eigenvalues().transpose() << "\n"
          << "Eigenvectors (columns):\n" << solver.eigenvectors() << "\n";


      // check if M is symmetric
      EXPECT_TRUE(root_M.isApprox(root_M.transpose()))
          << "Root M is not symmetric at angle " << theta << '\n'
          << "x: " << x.x() << ", " << x.y() << ", " << x.z() << '\n'
          << "root M: " << root_M << '\n';

      // check if M is SPD
      Eigen::LLT<Eigen::MatrixXd> llt(root_M.block<2, 2>(0, 0));
      EXPECT_FALSE(llt.info() == Eigen::NumericalIssue)
          << "root M is not SPD at angle " << theta << '\n'
          << "x: " << x.x() << ", " << x.y() << ", " << x.z() << '\n'
          << "root M: " << root_M << '\n'
          << "llt info: " << llt.info();

      Eigen::Vector3d root_f_direction = root_f.head<3>().normalized();

      EXPECT_GT(root_f_direction.dot(goal_direction), 0.7)
          << "Root force does not point towards the goal at angle " << theta
          << '\n'
          << "x: " << x.x() << ", " << x.y() << ", " << x.z() << '\n'
          << "goal direction: " << goal_direction.x() << ", "
          << goal_direction.y() << ", " << goal_direction.z() << '\n'
          << "root_f direction: " << root_f_direction.x() << ", "
          << root_f_direction.y() << ", " << root_f_direction.z() << '\n'
          << "goal f: " << root_f.x() << ", " << root_f.y() << ", "
          << root_f.z() << '\n';

      PositionAttractor<Position2D>::DomainVec obstacle_f;
      PositionAttractor<Position2D>::MatrixM obstacle_M;
      repulsor.pullback(obstacle_f, obstacle_M);
      Eigen::Vector3d obstacle_f_direction = Eigen::Vector3d::Zero();
      obstacle_f_direction.head<2>() = obstacle_f.normalized();

      // obstacle f should point away from obstacle, goal f towards the goal
      EXPECT_NEAR(obstacle_f_direction.dot(obstacle_direction), -1.0, 1e-6)
          << "Obstacle force does not point away from obstacle at angle "
          << theta << '\n'
          << "x: " << x.x() << ", " << x.y() << ", " << x.z() << '\n'
          << "obstacle direction: " << obstacle_direction.x() << ", "
          << obstacle_direction.y() << ", " << obstacle_direction.z() << '\n'
          << "obstacle_f direction: " << obstacle_f_direction.x() << ", "
          << obstacle_f_direction.y() << ", " << obstacle_f_direction.z()
          << '\n'
          << "obstacle f: " << obstacle_f.x() << ", " << obstacle_f.y() << '\n';

      PositionRepulsor<Position2D>::DomainVec goal_f;
      PositionRepulsor<Position2D>::MatrixM goal_M;
      attractor.pullback(goal_f, goal_M);
      Eigen::Vector3d goal_f_direction = Eigen::Vector3d::Zero();
      goal_f_direction.head<2>() = goal_f.normalized();

      EXPECT_NEAR(goal_f_direction.dot(goal_direction), 1.0, 1e-6)
          << "Goal force does not point towards the goal at angle " << theta
          << '\n'
          << "x: " << x.x() << ", " << x.y() << ", " << x.z() << '\n'
          << "goal direction: " << goal_direction.x() << ", "
          << goal_direction.y() << ", " << goal_direction.z() << '\n'
          << "goal_f direction: " << goal_f_direction.x() << ", "
          << goal_f_direction.y() << ", " << goal_f_direction.z() << '\n'
          << "goal f: " << goal_f.x() << ", " << goal_f.y() << '\n';

      Position2DMap::DomainVec position_map_f;
      Position2DMap::MatrixM position_map_M;
      position_map.pullback(position_map_f, position_map_M);
      Eigen::Vector3d position_map_f_direction =
          position_map_f.head<3>().normalized();

      EXPECT_GT(position_map_f_direction.dot(goal_direction), 0.7)
          << "Position map force does not point towards the goal at angle "
          << theta << '\n'
          << "x: " << x.x() << ", " << x.y() << ", " << x.z() << '\n'
          << "goal direction: " << goal_direction.x() << ", "
          << goal_direction.y() << ", " << goal_direction.z() << '\n'
          << "position_map_f direction: " << position_map_f_direction.x()
          << ", " << position_map_f_direction.y() << ", "
          << position_map_f_direction.z() << '\n'
          << "goal f: " << position_map_f.x() << ", " << position_map_f.y()
          << '\n';
    }
  }
}

TEST(RMPLeafTest, RepulsorAttractorObstacleProximityDirectionTest) {
  Root<SE3> root;
  auto& position_map = root.addChild<Position2DMap>();

  Eigen::Vector2d c(0.0, 0.0);
  position_map.addChild<PositionRepulsor<Position2D>>(c);

  Eigen::Vector2d goal(3.0, 3.0);
  position_map.addChild<PositionAttractor<Position2D>>(goal);

  double start_radius = 0.1;
  double end_radius = 0.5;
  double radius_increment = 0.1;

  for (double radius = start_radius; radius < end_radius;
       radius += radius_increment) {

    double angle_increment = 0.1;

    for (double theta = 0.0; theta < (2 * M_PI); theta += angle_increment) {
      double xi_c = c(0) + cos(theta) * radius;
      double yi_c = c(1) + sin(theta) * radius;

      Root<SE3>::DomainVec x(xi_c, yi_c, 0, 0, 0, 0);
      auto x_dot = Root<SE3>::DomainVec::Zero();

      Eigen::Vector3d obstacle_direction =
          (Eigen::Vector3d(c.x(), c.y(), 0.0) - x.head<3>()).normalized();

      Root<SE3>::DomainVec a = root.solve(x, x_dot);
      Eigen::Vector3d push_direction = a.head<3>().normalized();

      Root<SE3>::DomainVec root_f;
      Root<SE3>::MatrixM root_M;
      root.pullback(root_f, root_M);

      // expect it to push away from the obstacle
      EXPECT_LT(push_direction.dot(obstacle_direction), 0.0)
          << "Push direction does not point away from the obstacle at angle "
          << theta << '\n'
          << "x: " << x.x() << ", " << x.y() << ", " << x.z() << '\n'
          << "a: " << a.x() << ", " << a.y() << ", " << a.z() << '\n'
          << "obstacle direction: " << obstacle_direction.x() << ", "
          << obstacle_direction.y() << ", " << obstacle_direction.z() << '\n'
          << "push direction: " << push_direction.x() << ", "
          << push_direction.y() << ", " << push_direction.z() << '\n'
          << "root f: " << root_f.x() << ", " << root_f.y() << ", "
          << root_f.z() << '\n'
          << "root M: " << root_M << '\n';
    }
  }
}
