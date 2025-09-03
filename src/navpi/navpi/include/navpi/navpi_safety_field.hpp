#ifndef NAVPI_SAFETY_FIELD_H
#define NAVPI_SAFETY_FIELD_H

#include <pcl/point_types.h>

class SafetyField
{
public:
  SafetyField() = default;

  SafetyField(Eigen::Matrix<double, 3, 1> min_p, Eigen::Matrix<double, 3, 1> max_p)
    : m_min_p(min_p)
    , m_max_p(max_p)
  {
  }

  virtual ~SafetyField() = default;


  bool containsPoint(pcl::PointXYZ p) const
  {
    // clang-format off
    return (p.x >= m_min_p.x() &&
            p.y >= m_min_p.y() &&
            p.z >= m_min_p.z() &&
            p.x <= m_max_p.x() &&
            p.y <= m_max_p.y() &&
            p.z <= m_max_p.z());
    // clang-format on
  }

  Eigen::Matrix<double, 3, 1> getMinP() const { return m_min_p; }
  Eigen::Matrix<double, 3, 1> getMaxP() const { return m_max_p; }
  void triggerSafety() { m_safety_triggerd = true; }
  void releaseSafety() { m_safety_triggerd = false; }
  bool isSafetyTriggered() const { return m_safety_triggerd; }


private:
  Eigen::Matrix<double, 3, 1> m_min_p;
  Eigen::Matrix<double, 3, 1> m_max_p;
  bool m_safety_triggerd;
};

#endif /* NAVPI_SAFETY_FIELD_H */
