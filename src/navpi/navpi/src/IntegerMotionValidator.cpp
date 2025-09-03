#include "ompl/util/Exception.h"
#include <navpi/IntegerMotionValidator.h>
#include <queue>

void IntegerMotionValidator::defaultSettings()
{
  stateSpace_ = si_->getStateSpace().get();
  if (stateSpace_ == nullptr)
    throw ompl::Exception("No state space for motion validator");
}

bool IntegerMotionValidator::checkMotion(const ob::State* s1,
                                         const ob::State* s2,
                                         std::pair<ob::State*, double>& lastValid) const
{
  /* assume motion starts in a valid configuration so s1 is valid */

  bool result = true;
  int nd      = stateSpace_->validSegmentCount(s1, s2);

  if (nd > 1)
  {
    /* temporary storage for the checked state */
    ob::State* test = si_->allocState();

    for (int j = 1; j < nd; ++j)
    {
      stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
      if (!si_->isValid(test))
      {
        lastValid.second = (double)(j - 1) / (double)nd;
        if (lastValid.first != nullptr)
          stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
        result = false;
        break;
      }
    }
    si_->freeState(test);
  }

  if (result)
    if (!si_->isValid(s2))
    {
      lastValid.second = (double)(nd - 1) / (double)nd;
      if (lastValid.first != nullptr)
        stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
      result = false;
    }

  if (result)
    valid_++;
  else
    invalid_++;

  return result;
}

bool IntegerMotionValidator::checkMotion(const ob::State* s1, const ob::State* s2) const
{
  return true;
  /* assume motion starts in a valid configuration so s1 is valid */
  if (!si_->isValid(s2))
  {
    invalid_++;
    return false;
  }

  bool result = true;
  int nd      = stateSpace_->validSegmentCount(s1, s2);

  /* initialize the queue of test positions */
  std::queue<std::pair<int, int>> pos;
  if (nd >= 2)
  {
    pos.emplace(1, nd - 1);

    /* temporary storage for the checked state */
    ob::State* test = si_->allocState();

    /* repeatedly subdivide the path segment in the middle (and check the middle) */
    while (!pos.empty())
    {
      std::pair<int, int> x = pos.front();

      int mid = (x.first + x.second) / 2;
      stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, test);

      if (!si_->isValid(test))
      {
        result = false;
        break;
      }

      pos.pop();

      if (x.first < mid)
        pos.emplace(x.first, mid - 1);
      if (x.second > mid)
        pos.emplace(mid + 1, x.second);
    }

    si_->freeState(test);
  }

  if (result)
    valid_++;
  else
    invalid_++;

  return result;
}
