#pragma once

#include <optional>
#include <rumps/types.hpp>
#include <memory>
#include <type_traits>
#include <utility>

namespace rumps {

template <typename DomainT, typename ChildT = ChildInterface<DomainT>, std::size_t N = 10000>
class DynamicNodeContainer : public ChildContainer<DomainT> {
public:
  DEFINE_RMP_TYPES(DomainT, DomainT);

  DynamicNodeContainer() = default;
  ~DynamicNodeContainer() = default;
  DynamicNodeContainer(const DynamicNodeContainer&) = delete;
  DynamicNodeContainer& operator=(const DynamicNodeContainer&) = delete;
  DynamicNodeContainer(DynamicNodeContainer&&) = default;
  DynamicNodeContainer& operator=(DynamicNodeContainer&&) = default;

  template <typename T, typename... Args>
  T& addChild(Args&&... args) {
    static_assert(std::is_base_of_v<Child, T>, "T must be derived of Child type");
    auto ptr = std::make_unique<T>(std::forward<Args>(args)...);
    T& ref = *ptr;
    m_children.emplace_back(std::move(ptr));
    return ref;
  }
  size_t size() const { return m_children.size(); }
  Child &getChild(size_t idx) { return *m_children.at(idx); }
  void clearChildren() { m_children.clear(); }
private:
  std::vector<std::unique_ptr<Child>> m_children;
};

template <typename DomainT, typename ChildT, std::size_t N>
class StaticNodeContainer : public ChildContainer<DomainT> {
public:
  DEFINE_RMP_TYPES(DomainT, DomainT);

  StaticNodeContainer() = default;
  ~StaticNodeContainer() = default;
  StaticNodeContainer(const StaticNodeContainer&) = delete;
  StaticNodeContainer& operator=(const StaticNodeContainer&) = delete;
  StaticNodeContainer(StaticNodeContainer&&) = default;
  StaticNodeContainer& operator=(StaticNodeContainer&&) = default;

  template <typename T, typename... Args>
  T& addChild(Args&&... args) {
    assert(m_curr_idx < N);
    static_assert(std::is_same<ChildT, T>(), "T must be the ChildT type");
    m_children.at(m_curr_idx).emplace(std::forward<Args>(args)...);
    return *m_children.at(m_curr_idx++);
  }
  size_t size() const { return m_curr_idx; }
  Child &getChild(size_t idx) { return *m_children.at(idx); }
  void clearChildren() { m_curr_idx = 0; }
private:
  std::array<std::optional<ChildT>, N> m_children;
  size_t m_curr_idx = 0;
};

}
