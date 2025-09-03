/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Jan Oberlaender, Sebastian Klemm */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_RRTSTAR_CONNECT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_RRTSTAR_CONNECT_

#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/geometric/planners/PlannerIncludes.h>

#include <limits>
#include <utility>
#include <vector>


namespace ompl {

namespace geometric {

/**
   @anchor gRRTstarConnect
   @par Short description
   \ref gRRTstarCOnnect "RRT*Connect" (optimal RRTConnect) is
   an asymptotically-optimal incremental sampling-based motion
   planning algorithm. \ref gRRTstarConnect "RRT*Connect"
   algorithm is guaranteed to converge to an optimal solution,
   while its running time is guaranteed to be a constant
   factor of the running time of the \ref gRRT "RRT". The
   notion of optimality is with respect to the distance
   function defined on the state space we are operating
   on. See ompl::base::Goal::setMaximumPathLength() for how to
   set the maximally allowed path length to reach the goal.
   If a solution path that is shorter than
   ompl::base::Goal::getMaximumPathLength() is found, the
   algorithm terminates before the elapsed time.  @par
   External documentation S. Karaman and E. Frazzoli,
   Sampling-based Algorithms for Optimal Motion Planning,
   International Journal of Robotics Research, Vol 30, No 7,
   2011.  http://arxiv.org/abs/1105.1186
*/

/** \brief Optimal Rapidly-exploring Random Trees */
class RRTstarConnect : public base::Planner
{
public:
  /** \brief PlannerDataVertex tag indicating a start tree vertex. */
  static const int START_TREE_VERTEX;
  /** \brief PlannerDataVertex tag indicating a goal tree vertex. */
  static const int GOAL_TREE_VERTEX;

  RRTstarConnect(const base::SpaceInformationPtr& si);

  virtual ~RRTstarConnect();

  virtual void getPlannerData(base::PlannerData& data) const;

  virtual base::PlannerStatus solve(const base::PlannerTerminationCondition& ptc);

  virtual void clear();

  /** \brief Set the range the planner is supposed to use.

      This parameter greatly influences the runtime of the
      algorithm. It represents the maximum length of a
      motion to be added in the tree of motions. */
  void setRange(double distance) { maxDistance_ = distance; }

  /** \brief Get the range the planner is using */
  double getRange() const { return maxDistance_; }

  /** \brief Set a different nearest neighbors datastructure */
  template <template <typename T> class NN>
  void setNearestNeighbors()
  {
    nnStart_.reset(new NN<Motion*>());
    nnGoal_.reset(new NN<Motion*>());
  }

  /** \brief Option that delays collision checking procedures.
      When it is enabled, all neighbors are sorted by cost. The
      planner then goes through this list, starting with the lowest
      cost, checking for collisions in order to find a parent. The planner
      stops iterating through the list when a collision free parent is found.
      This prevents the planner from collsion checking each neighbor, reducing
      computation time in scenarios where collision checking procedures are expensive.*/
  void setDelayCC(bool delayCC) { delayCC_ = delayCC; }

  /** \brief Get the state of the delayed collision checking option */
  bool getDelayCC() const { return delayCC_; }

  /** \brief Option that alternates between trees.
      When it is enabled, sampling alternates between the
      two trees.  When it is disabled, the sample is always
      attached to the nearest tree.*/
  void setAlternateTrees(bool alternateTrees) { alternateTrees_ = alternateTrees; }

  /** \brief Get the state of the delayed collision checking option */
  bool getAlternateTrees() const { return alternateTrees_; }

  /** \brief Controls whether the tree is pruned during the search. */
  void setPrune(const bool prune) { prune_ = prune; }

  /** \brief Get the state of the pruning option. */
  bool getPrune() const { return prune_; }

  /** \brief Set the percentage threshold (between 0 and 1) for pruning the tree. If the new tree
     has removed at least this percentage of states, the tree will be finally pruned. */
  void setPruneStatesImprovementThreshold(const double pp) { pruneStatesThreshold_ = pp; }

  /** \brief Get the current prune states percentage threshold parameter. */
  double getPruneStatesImprovementThreshold() const { return pruneStatesThreshold_; }

  virtual void setup();

  ///////////////////////////////////////
  // Planner progress property functions
  std::string getIterationCount() const { return boost::lexical_cast<std::string>(iterations_); }
  std::string getBestCost() const { return boost::lexical_cast<std::string>(bestCost_); }

protected:
  /** \brief Representation of a motion */
  class Motion
  {
  public:
    /** \brief Constructor that allocates memory for the state. This constructor automatically
     * allocates memory for \e state, \e cost, and \e incCost */
    Motion(const base::SpaceInformationPtr& si)
      : state(si->allocState())
      , parent(NULL)
      , connection(NULL)
    {
    }

    ~Motion() {}

    /** \brief The state contained by the motion */
    base::State* state;

    /** \brief The parent motion in the exploration tree */
    Motion* parent;

    /** \brief The cost up to this motion */
    base::Cost cost;

    /** \brief The incremental cost of this motion's parent to this motion (this is stored to save
     * distance computations in the updateChildCosts() method) */
    base::Cost incCost;

    /** \brief The set of motions descending from the current motion */
    std::vector<Motion*> children;

    /** \brief Connection to the other tree */
    Motion* connection;
  };

  /** \brief Free the memory allocated by this planner */
  void freeMemory();

  // For sorting a list of costs and getting only their sorted indices
  struct CostIndexCompare
  {
    CostIndexCompare(const std::vector<base::Cost>& costs, const base::OptimizationObjective& opt)
      : costs_(costs)
      , opt_(opt)
    {
    }
    bool operator()(unsigned i, unsigned j) { return opt_.isCostBetterThan(costs_[i], costs_[j]); }
    const std::vector<base::Cost>& costs_;
    const base::OptimizationObjective& opt_;
  };

  /** \brief Like MotionValidator::checkMotion(), but also checks \a s1 for validity */
  bool checkMotionAndStartState(const base::State* s1, const base::State* s2) const;

  /** \brief Compute distance between motions (actually distance between contained states) */
  double distanceFunction(const Motion* a, const Motion* b) const
  {
    return si_->distance(a->state, b->state);
  }

  /** \brief Removes the given motion from the parent's child list */
  void removeFromParent(Motion* m);

  /** \brief Updates the cost of the children of this node if the cost up to this node has changed
      Returns \c true if the best solution was updated in the process. */
  bool updateChildCosts(Motion* m, bool fromStart);

/** \brief A nearest-neighbor datastructure representing a tree of motions */
#if OMPL_VERSION_VALUE < 1002001
  typedef boost::shared_ptr<NearestNeighbors<Motion*>> TreeData;
#else
  typedef std::shared_ptr<NearestNeighbors<Motion*>> TreeData;
#endif

  /** \brief Prunes all those states which estimated total cost is higher than pruneTreeCost.
      Returns the number of motions pruned. Depends on the parameter set by
     setPruneStatesImprovementThreshold() */
  int pruneTree(const base::Cost pruneTreeCost);

  int pruneTree(const TreeData& nn, const base::Cost pruneTreeCost);

  /** \brief Deletes (frees memory) the motion and its children motions. */
  void deleteBranch(Motion* motion);

  /** \brief Computes the Cost To Go heuristically as the cost to come from start to motion plus
       the cost to go from motion to goal. If \e shortest is true, the estimated cost to come
       start-motion is given. Otherwise, this cost to come is the current motion cost.
       For a motion from the goal tree, returns the Cost To Go towards the start. */
  base::Cost costToGo(const Motion* motion, const bool shortest = true) const;

  /** \brief State sampler */
  base::StateSamplerPtr sampler_;

  /** \brief The start tree */
  TreeData nnStart_;

  /** \brief The goal tree */
  TreeData nnGoal_;

  /** \brief The maximum length of a motion to be added to a tree */
  double maxDistance_;

  /** \brief The random number generator */
  RNG rng_;

  /** \brief Option to delay and reduce collision checking within iterations */
  bool delayCC_;

  /** \brief Option to alternate between trees */
  bool alternateTrees_;

  /** \brief Objective we're optimizing */
  base::OptimizationObjectivePtr opt_;

  /** \brief Stores the Motion containing the last added initial start state. */
  Motion* startMotion_;

  /** \brief Stores the Motion containing the last added goal state. */
  Motion* goalMotion_;

  /** \brief If this value is set to true, tree pruning will be enabled. */
  bool prune_;

  /** \brief The tree is only pruned is the percentage of states to prune is above this threshold
   * (between 0 and 1). */
  double pruneStatesThreshold_;

  struct PruneScratchSpace
  {
    std::vector<Motion*> newTree, toBePruned, candidates;
  } pruneScratchSpace_;

  //////////////////////////////
  // Planner progress properties
  /** \brief Number of iterations the algorithm performed */
  unsigned int iterations_;
  /** \brief Best connection found so far (always on the start tree) */
  Motion* bestConnection_;
  /** \brief Overall best cost (bestStartCost_ + bestGoalCost_ + bestConnection_'s incremental cost)
   */
  base::Cost bestCost_;
};
} // namespace geometric
} // namespace ompl

#endif
