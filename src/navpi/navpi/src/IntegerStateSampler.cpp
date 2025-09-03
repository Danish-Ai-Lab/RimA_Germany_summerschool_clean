#include <navpi/IntegerStateSampler.h>
#include <vdb_mapping/VDBMapping.hpp>


IntegerStateSampler::IntegerStateSampler(const ob::StateSpace* space,
                                         std::shared_ptr<MapHandle> map_handle)
  : ob::StateSampler(space)
  , map_handle_(map_handle)
{
  bbox_ = map_handle->getValueGrid()->evalActiveVoxelBoundingBox();
  std::cout << "Map Dimension: " << bbox_ << std::endl;
}

void IntegerStateSampler::sampleUniform(ob::State* state)
{
  openvdb::Coord sample(rng_.uniformInt(bbox_.min().x(), bbox_.max().x()),
                        rng_.uniformInt(bbox_.min().y(), bbox_.max().y()),
                        rng_.uniformInt(bbox_.min().z(), bbox_.max().z()));

  state->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(0)->value =
    sample.x();
  state->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(1)->value =
    sample.y();
  state->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(2)->value =
    sample.z();

  space_->enforceBounds(state);
}
void IntegerStateSampler::sampleUniformNear(ob::State* state,
                                            const ob::State* near,
                                            double distance)
{
  std::cout << "samle near" << std::endl;
}

void IntegerStateSampler::sampleGaussian(ob::State* state, const ob::State* mean, double stdDev)
{
  std::cout << "samle gaussian" << std::endl;
}
