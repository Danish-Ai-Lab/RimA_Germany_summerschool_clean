#ifndef PLANNER_EVAL_INTEGER_STATE_SPACE_H_INCLUDED
#define PLANNER_EVAL_INTEGER_STATE_SPACE_H_INCLUDED

#include <iostream>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <vdb_mapping/VDBMapping.hpp>

namespace ob = ompl::base;

class IntegerStateSpace : public ob::CompoundStateSpace
{
public:
  IntegerStateSpace();
  IntegerStateSpace(openvdb::CoordBBox bbox);
};

#endif // !PLANNER_EVAL_INTEGER_STATE_SPACE_H_INCLUDED
