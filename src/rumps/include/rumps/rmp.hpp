#pragma once

#include <Eigen/Core>
#include <Eigen/QR>

#include <rumps/types.hpp>
#include <rumps/containers.hpp>

namespace rumps {


template <typename DomainT, typename RangeT = DomainT, typename ContainerT = DynamicNodeContainer<RangeT>>
class Node : public ChildInterface<DomainT> {
public:
  DEFINE_RMP_TYPES(DomainT, RangeT);

  virtual DomainVec q() const { return m_q; }
  virtual DomainVec q_dot() const { return m_q_dot; }
  virtual RangeVec x() const { return m_x; }
  virtual RangeVec x_dot() const { return m_x_dot; }
  virtual MatrixJ J_() const { return m_J; }
  virtual MatrixJ J_dot() const { return m_J_dot; }
  virtual DomainVec f() const { return m_f; }
  virtual DomainMatrixM M() const { return m_M; }

  virtual void pushforward(const DomainVec &q,
                           const DomainVec &q_dot) override {
    m_q = q;
    m_q_dot = q_dot;

    J(m_q, m_J);
    J_dot(m_q, m_q_dot, m_J_dot);

    psi(m_q, m_x);
    m_x_dot.noalias() = m_J * m_q_dot;

    for (size_t i = 0; i < size(); ++i) {
      getChild(i).pushforward(m_x, m_x_dot);
    }
  }

  virtual void pullback(DomainVec &f, DomainMatrixM &M) override {
    RangeVec f_range = RangeVec::Zero();
    RangeMatrixM M_range = RangeMatrixM::Zero();

    compute(m_x, m_x_dot, f_range, M_range);

    f.noalias() = m_J.transpose() * (f_range - M_range * m_J_dot * m_q_dot);
    M.noalias() = m_J.transpose() * M_range * m_J;

    m_f = f;
    m_M = M;
  }

  virtual void compute([[maybe_unused]] const RangeVec &x,
                       [[maybe_unused]] const RangeVec &x_dot, RangeVec &f,
                       RangeMatrixM &M) {
    f.setZero();
    M.setZero();

    for (size_t i = 0; i < size(); ++i) {
      RangeVec child_f = RangeVec::Zero();
      RangeMatrixM child_M = RangeMatrixM::Zero();
      getChild(i).pullback(child_f, child_M);

      f += child_f;
      M += child_M;
    }
  }

  virtual void psi([[maybe_unused]] const DomainVec &q, RangeVec& x) const {
    x = q.template head<Range::Dim>();
  }

  virtual void J([[maybe_unused]] const DomainVec &q, MatrixJ& J) const {
    J = MatrixJ::Identity();
  }

  virtual void J_dot([[maybe_unused]] const DomainVec &q,
                        [[maybe_unused]] const DomainVec &q_dot, MatrixJ& J_dot) const {
    J_dot.setZero();
  }

  template <typename T, typename... Args>
  T& addChild(Args&&... args) {
    return m_child_container.template addChild<T>(std::forward<Args>(args)...);
  }
  size_t size() const { return m_child_container.size(); }
  Child &getChild(size_t idx) { return m_child_container.getChild(idx); }
  void clearChildren() { m_child_container.clearChildren(); }

protected:
  DomainVec m_q, m_q_dot;
  RangeVec m_x, m_x_dot;
  MatrixJ m_J, m_J_dot;

  DomainVec m_f;
  DomainMatrixM m_M;

  ContainerT m_child_container;
};

template <typename DomainT, typename RangeT = DomainT, typename ContainerT = DynamicNodeContainer<RangeT>>
class ParallelNode : public Node<DomainT, RangeT, ContainerT> {
public:
  DEFINE_RMP_TYPES(DomainT, RangeT);

  virtual void pushforward(const DomainVec &q,
                           const DomainVec &q_dot) override {
    this->m_q = q;
    this->m_q_dot = q_dot;

    this->J(this->m_q, this->m_J);
    this->J_dot(this->m_q, this->m_q_dot, this->m_J_dot);

    this->psi(this->m_q, this->m_x);
    this->m_x_dot.noalias() = this->m_J * this->m_q_dot;

#pragma omp parallel for
    for (size_t i = 0; i < this->size(); ++i) {
      this->getChild(i).pushforward(this->m_x, this->m_x_dot);
    }
  }

  virtual void compute([[maybe_unused]] const RangeVec &x,
                       [[maybe_unused]] const RangeVec &x_dot, RangeVec &f,
                       RangeMatrixM &M) override {
    f.setZero();
    M.setZero();

#pragma omp parallel
    {
      RangeVec f_private = RangeVec::Zero();
      RangeMatrixM M_private = RangeMatrixM::Zero();

#pragma omp for nowait
      for (size_t i = 0; i < this->size(); ++i) {
        RangeVec child_f = RangeVec::Zero();
        RangeMatrixM child_M = RangeMatrixM::Zero();
        this->getChild(i).pullback(child_f, child_M);

        f_private.noalias() += child_f;
        M_private.noalias() += child_M;
      }

#pragma omp critical
      {
        f.noalias() += f_private;
        M.noalias() += M_private;
      }
    }
  }
};

template <typename DomainT, typename ContainerT = DynamicNodeContainer<DomainT>>
class Root : public Node<DomainT, DomainT, ContainerT> {
public:
  DEFINE_RMP_TYPES(DomainT, DomainT);

  // qualify parent functions
  using ChildInterface<DomainT>::pushforward;
  using ChildInterface<DomainT>::pullback;

  virtual void psi(const DomainVec &q, RangeVec& x) const override {
    x.noalias() = q;
  }

  DomainVec a() const { return m_a; }

  DomainVec solve(const DomainVec &q, const DomainVec &q_dot) {
    pushforward(q, q_dot);
    DomainVec f = DomainVec::Zero();
    MatrixM M = MatrixM::Zero();
    pullback(f, M);

    m_a = M.completeOrthogonalDecomposition().pseudoInverse() * f;
    return m_a;
  }

protected:
  DomainVec m_a;
};
} // namespace rumps
