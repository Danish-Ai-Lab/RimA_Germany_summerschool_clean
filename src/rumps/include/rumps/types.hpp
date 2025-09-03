#pragma once

#include <Eigen/Eigen>

namespace rumps {

#define DEFINE_RMP_DOMAIN_TYPES(DomainT)                                       \
  using Domain = DomainT;                                                      \
  using DomainVec = typename Domain::Vec;                                      \
  using DomainMatrixM = typename Domain::MatrixM;                              \
  using MatrixM = Eigen::Matrix<double, Domain::Dim, Domain::Dim>

#define DEFINE_RMP_RANGE_TYPES(RangeT)                                         \
  using Range = RangeT;                                                        \
  using RangeVec = typename Range::Vec;                                        \
  using RangeMatrixM = typename Range::MatrixM;                                \
  using Child = ChildInterface<Range>

#define DEFINE_RMP_TYPES(DomainT, RangeT)                                      \
  DEFINE_RMP_DOMAIN_TYPES(DomainT);                                            \
  DEFINE_RMP_RANGE_TYPES(RangeT);                                              \
  using MatrixJ = Eigen::Matrix<double, Range::Dim, Domain::Dim>

template <typename DomainT> struct ChildInterface {
  DEFINE_RMP_DOMAIN_TYPES(DomainT);

  virtual void pushforward(const DomainVec &q, const DomainVec &q_dot) = 0;
  virtual void pullback(DomainVec &f, DomainMatrixM &M) = 0;
};

template <typename DomainT> class ChildContainer {
public:
  DEFINE_RMP_RANGE_TYPES(DomainT);

  virtual size_t size() const = 0;
  virtual Child &getChild(size_t idx) = 0;
  virtual void clearChildren() = 0;
};

} // namespace rumps
