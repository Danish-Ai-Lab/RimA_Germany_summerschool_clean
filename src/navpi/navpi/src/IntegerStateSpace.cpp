#include <navpi/IntegerStateSpace.h>


IntegerStateSpace::IntegerStateSpace()
{
  std::cout << " constructing state space" << std::endl;
  this->setName("IntegerStateSpace");
  this->type_ = ob::STATE_SPACE_REAL_VECTOR;
  this->addSubspace(ob::StateSpacePtr(new ob::DiscreteStateSpace(-10000, 10000)), 1.0);
  this->addSubspace(ob::StateSpacePtr(new ob::DiscreteStateSpace(-10000, 10000)), 1.0);
  this->addSubspace(ob::StateSpacePtr(new ob::DiscreteStateSpace(0, 10000)), 1.0);
  std::cout << "done constructing state space" << std::endl;
}
IntegerStateSpace::IntegerStateSpace(openvdb::CoordBBox bbox)
//: ob::CompoundStateSpace()
{
  std::cout << " constructing state space" << std::endl;
  this->setName("IntegerStateSpace");
  this->type_ = ob::STATE_SPACE_REAL_VECTOR;
  std::cout << bbox << std::endl;
  this->addSubspace(ob::StateSpacePtr(new ob::DiscreteStateSpace(bbox.min().x(), bbox.max().x())),
                    1.0);
  this->addSubspace(ob::StateSpacePtr(new ob::DiscreteStateSpace(bbox.min().y(), bbox.max().y())),
                    1.0);
  this->addSubspace(ob::StateSpacePtr(new ob::DiscreteStateSpace(bbox.min().z(), bbox.max().z())),
                    1.0);
  std::cout << "done constructing state space" << std::endl;
}
