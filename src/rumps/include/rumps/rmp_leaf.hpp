#pragma once

#include "rmp.hpp"
#include "spaces.hpp"
#include <algorithm>

namespace rumps {

template <typename DomainT>
class DistanceLeaf : public Node<DomainT, ONE> {
public:
  DistanceLeaf(const typename DomainT::Vec &c = DomainT::Vec::Zero())
      : Node<DomainT, ONE>(), m_c(c) {}
  DEFINE_RMP_TYPES(DomainT, ONE);

  virtual void psi([[maybe_unused]] const DomainVec &q, RangeVec& x) const override {
    x(0) = (q - m_c).norm();
  }

  virtual void J([[maybe_unused]] const DomainVec &q, MatrixJ& J) const override {
    DomainVec diff = q - m_c;
    J.noalias() = 1.0 / diff.norm() * diff.transpose();
  }

  virtual void J_dot([[maybe_unused]] const DomainVec &q,
                        [[maybe_unused]] const DomainVec &q_dot, MatrixJ& J_dot) const override {

    DomainVec diff = q - m_c;
    double norm = diff.norm();

    J_dot.noalias() = q_dot.transpose() *
           (pow(-1.0 / norm, 3.0) * diff * diff.transpose() +
            1.0 / norm *
                Eigen::Matrix<double, DomainT::Dim, DomainT::Dim>::Identity());
  }

  void setC(const DomainVec &c) {
    m_c = c;
  }

protected:
  DomainVec m_c;
};

template <typename DomainT>
class PositionRepulsor : public DistanceLeaf<DomainT> {
public:
  PositionRepulsor(const typename DomainT::Vec &c = DomainT::Vec::Zero(), double epsilon = 1.0,
                   double eta = 0.0, double alpha = 1e-5, int p = 4)
      : DistanceLeaf<DomainT>(c), m_epsilon(epsilon), m_eta(eta),
        m_alpha(alpha), m_p(p) {}
  DEFINE_RMP_TYPES(DomainT, ONE);

  virtual void compute(const RangeVec &x, const RangeVec &x_dot, RangeVec &f,
                       RangeMatrixM &M) override {
    double psi = x(0);
    double psi_dot = x_dot(0);

    double w = 1.0 / pow(psi, m_p);
    double grad_w = -m_p / pow(psi, m_p+1);

    // epsilon is the constant value when moving away from the obstacle
    double u = m_epsilon + std::min(0.0, psi_dot) * psi_dot;
    double g = w * u;
    double grad_u = 2 * std::min(0.0, psi_dot);
    double grad_Phi = m_alpha * w * grad_w;
    double xi = 0.5 * pow(psi_dot, 2) * u * grad_w;

    M(0, 0) = g + 0.5 * psi_dot * w * grad_u;
    M(0, 0) = std::clamp(M(0, 0), -1e5, 1e5);


    double Bx_dot = m_eta * g * psi_dot;
    f(0) = -grad_Phi - xi - Bx_dot;
    f(0) = std::clamp(f(0), -1e10, 1e10);
  }

private:
  double m_epsilon;
  double m_eta;
  double m_alpha;
  int m_p;
};


template <typename DomainT>
class PositionAttractor : public DistanceLeaf<DomainT> {
public:
  PositionAttractor(const typename DomainT::Vec &c = DomainT::Vec::Zero(),
                    double eta = 0.0, double alpha = 1e-5, double psi_thresh = 2.0)
      : DistanceLeaf<DomainT>(c), m_eta(eta),
        m_alpha(alpha), m_psi_thresh(psi_thresh) {}
  DEFINE_RMP_TYPES(DomainT, ONE);

  virtual void compute(const RangeVec &x,
                       const RangeVec &x_dot, RangeVec &f,
                       RangeMatrixM &M) override {
    double psi = x(0);
    double psi_dot = x_dot(0);

    psi = std::min(psi, m_psi_thresh);

    double g = 1.0;
    M(0, 0) = g;

    double grad_Phi = m_alpha * psi;

    double Bx_dot = m_eta * psi_dot;
    f(0) = -grad_Phi - Bx_dot;
    f(0) = std::clamp(f(0), -1e10, 1e10);
  }

private:
  double m_eta;
  double m_alpha;
  double m_psi_thresh;
};


template <typename DomainT>
class TikhonovRegularizer : public Node<DomainT> {
public:
  TikhonovRegularizer(const double lambda = 1.0) : Node<DomainT>(), m_lambda(lambda) {}
  DEFINE_RMP_TYPES(DomainT, DomainT);

  virtual void compute([[maybe_unused]] const RangeVec &x,
                       [[maybe_unused]] const RangeVec &x_dot, RangeVec &f,
                       RangeMatrixM &M) override {
    f = RangeVec::Zero();
    M = RangeMatrixM::Identity() * m_lambda;
  }

  void setLambda(double lambda) { m_lambda = lambda; }

private:
  double m_lambda;
};

template <typename DomainT>
class Damper : public Node<DomainT> {
public:
  Damper(const double lambda = 0.01) : Node<DomainT>(), m_lambda(lambda) {}
  DEFINE_RMP_TYPES(DomainT, DomainT);

  virtual void compute([[maybe_unused]] const RangeVec &x,
                       [[maybe_unused]] const RangeVec &x_dot, RangeVec &f,
                       RangeMatrixM &M) override {
    RangeMatrixM B = RangeMatrixM::Identity() * m_lambda;
    f = -B * x_dot;
    M = B;
  }

  void setLambda(double lambda) { m_lambda = lambda; }

private:
  double m_lambda;
};

} // namespace rumps
