#pragma once


#include <Eigen/Core>

namespace rumps {
struct SE3 {
  // x, y, z, roll, pitch, yaw
  static constexpr int Dim = 6;
  using Vec = Eigen::Matrix<double, Dim, 1>;
  using MatrixM = Eigen::Matrix<double, Dim, Dim>;
};

struct SE2 {
  // x, y, yaw
  static constexpr int Dim = 3;
  using Vec = Eigen::Matrix<double, Dim, 1>;
  using MatrixM = Eigen::Matrix<double, Dim, Dim>;
};

struct Position3D {
  // x, y, z
  static constexpr int Dim = 3;
  using Vec = Eigen::Matrix<double, Dim, 1>;
  using MatrixM = Eigen::Matrix<double, Dim, Dim>;
};

struct Position2D {
  // x, y
  static constexpr int Dim = 2;
  using Vec = Eigen::Matrix<double, Dim, 1>;
  using MatrixM = Eigen::Matrix<double, Dim, Dim>;
};

struct ONE {
  // just one dimensional
  static constexpr int Dim = 1;
  using Vec = Eigen::Matrix<double, Dim, 1>;
  using MatrixM = Eigen::Matrix<double, Dim, Dim>;
};

struct Diff {
  // x, yaw
  static constexpr int Dim = 2;
  using Vec = Eigen::Matrix<double, Dim, 1>;
  using MatrixM = Eigen::Matrix<double, Dim, Dim>;
};
}
