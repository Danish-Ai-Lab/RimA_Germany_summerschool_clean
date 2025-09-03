#pragma once

#include "rmp.hpp"
#include "spaces.hpp"

namespace rumps {

class Position3DMap : public Node<SE3, Position3D> {
public:
  virtual void psi([[maybe_unused]] const DomainVec &q, RangeVec& x) const override {
    x.noalias() = q.head<3>();
  }

  virtual void J([[maybe_unused]] const DomainVec &q, MatrixJ& J) const override {
    J.setZero();
    // only care about position, so first 3x3 block is identity
    J.block<3, 3>(0, 0).noalias() = Eigen::Matrix<double, 3, 3>::Identity();
  }
};

class Position2DMap : public Node<SE3, Position2D> {
public:
  virtual void psi([[maybe_unused]] const DomainVec &q, RangeVec& x) const override {
    x = q.head<2>();
  }

  virtual void J([[maybe_unused]] const DomainVec &q, MatrixJ& J) const override {
    J.setZero();
    // only care about position, so first 2x2 block is identity
    J.block<2, 2>(0, 0).noalias() = Eigen::Matrix<double, 2, 2>::Identity();
  }
};

}
