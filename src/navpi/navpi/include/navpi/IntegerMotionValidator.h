#ifndef PLANNER_EVAL_INTEGER_MOTION_VALIDATOR_H_INCLUDED_
#define PLANNER_EVAL_INTEGER_MOTION_VALIDATOR_H_INCLUDED_

#include "ompl/base/MotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include <iostream>

namespace ob = ompl::base;
/** \brief A motion validator that only uses the state validity checker. Motions are checked for
 * validity at a specified resolution. */
class IntegerMotionValidator : public ob::MotionValidator
{
public:
  /** \brief Constructor */
  IntegerMotionValidator(ob::SpaceInformation* si)
    : ob::MotionValidator(si)
  {
    std::cout << " Motion validator build" << std::endl;
    defaultSettings();
  }

  /** \brief Constructor */
  IntegerMotionValidator(const ob::SpaceInformationPtr& si)
    : ob::MotionValidator(si)
  {
    std::cout << " Motion validator build" << std::endl;
    defaultSettings();
  }

  ~IntegerMotionValidator() override = default;

  bool checkMotion(const ob::State* s1, const ob::State* s2) const override;

  bool checkMotion(const ob::State* s1,
                   const ob::State* s2,
                   std::pair<ob::State*, double>& lastValid) const override;

private:
  ob::StateSpace* stateSpace_;

  void defaultSettings();
};
#endif
