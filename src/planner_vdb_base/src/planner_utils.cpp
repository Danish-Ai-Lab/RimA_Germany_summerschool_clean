#include "planner_vdb_base/planner_utils.h"
#include <cmath>
#include <limits>


namespace planner_vdb_base {

bool doubleEquals(double a, double b)
{
  // return a == b;
  return (a == std::numeric_limits<double>::infinity() &&
          b == std::numeric_limits<double>::infinity()) ||
         std::fabs(a - b) < 0.0001f;
}

float degreeToRadian(float degree)
{
  return (degree * M_PI) / 180.0;
}
} // namespace planner_vdb_base