#ifndef PLANNER_EVAL_INTEGER_STATE_SAMPLER_H_INCLUDED
#define PLANNER_EVAL_INTEGER_STATE_SAMPLER_H_INCLUDED

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>

#include <navpi/IntegerStateSpace.h>
#include <navpi/MapHandle.h>

namespace ob = ompl::base;

class IntegerStateSampler : public ob::StateSampler
{
public:
  IntegerStateSampler(const ob::StateSpace* space, std::shared_ptr<MapHandle> map_handle);
  void sampleUniform(ob::State* state) override;
  void sampleUniformNear(ob::State* state, const ob::State* near, double distance) override;
  void sampleGaussian(ob::State* state, const ob::State* mean, double stdDev) override;

protected:
  std::shared_ptr<MapHandle> map_handle_;
  ompl::RNG rng_;
  openvdb::CoordBBox bbox_;
};


#endif // !PLANNER_EVAL_INTEGER_STATE_SAMPLER_H_INCLUDED
