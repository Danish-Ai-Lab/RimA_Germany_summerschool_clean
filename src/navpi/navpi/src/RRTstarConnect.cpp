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

#include "navpi/RRTstarConnect.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <functional>
#include <limits>
#include <memory>

const int ompl::geometric::RRTstarConnect::START_TREE_VERTEX = 0;

const int ompl::geometric::RRTstarConnect::GOAL_TREE_VERTEX = 1;

ompl::geometric::RRTstarConnect::RRTstarConnect(const base::SpaceInformationPtr& si)
  : base::Planner(si, "RRTstarConnect")
  , maxDistance_(0.0)
  , delayCC_(true)
  , alternateTrees_(false)
  , startMotion_(NULL)
  , goalMotion_(NULL)
  , prune_(false)
  , pruneStatesThreshold_(0.95)
  , iterations_(0)
  , bestConnection_(NULL)
  , bestCost_(std::numeric_limits<double>::quiet_NaN())
{
  specs_.approximateSolutions           = true;
  specs_.optimizingPaths                = true;
  specs_.canReportIntermediateSolutions = false;

  Planner::declareParam<double>(
    "range", this, &RRTstarConnect::setRange, &RRTstarConnect::getRange, "0.:1.:10000.");
  Planner::declareParam<bool>("delay_collision_checking",
                              this,
                              &RRTstarConnect::setDelayCC,
                              &RRTstarConnect::getDelayCC,
                              "0,1");
  Planner::declareParam<bool>("alternate_trees",
                              this,
                              &RRTstarConnect::setAlternateTrees,
                              &RRTstarConnect::getAlternateTrees,
                              "0,1");
  Planner::declareParam<bool>(
    "prune", this, &RRTstarConnect::setPrune, &RRTstarConnect::getPrune, "0,1");
  Planner::declareParam<double>("prune_states_threshold",
                                this,
                                &RRTstarConnect::setPruneStatesImprovementThreshold,
                                &RRTstarConnect::getPruneStatesImprovementThreshold,
                                "0.:.01:1.");

  addPlannerProgressProperty("iterations INTEGER",
                             std::bind(&RRTstarConnect::getIterationCount, this));
  addPlannerProgressProperty("best cost REAL", std::bind(&RRTstarConnect::getBestCost, this));
}

ompl::geometric::RRTstarConnect::~RRTstarConnect()
{
  freeMemory();
}

void ompl::geometric::RRTstarConnect::setup()
{
  Planner::setup();
  tools::SelfConfig sc(si_, getName());
  sc.configurePlannerRange(maxDistance_);
  if (!si_->getStateSpace()->hasSymmetricDistance() ||
      !si_->getStateSpace()->hasSymmetricInterpolate())
  {
    OMPL_WARN("%s requires a state space with symmetric distance and symmetric interpolation.",
              getName().c_str());
  }

#if OMPL_VERSION_VALUE < 1002001
  if (!nnStart_)
    nnStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
  if (!nnGoal_)
    nnGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
#else
  if (!nnStart_)
    nnStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
  if (!nnGoal_)
    nnGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
#endif
  {
    using namespace std::placeholders;
    nnStart_->setDistanceFunction(std::bind(&RRTstarConnect::distanceFunction, this, _1, _2));
    nnGoal_->setDistanceFunction(std::bind(&RRTstarConnect::distanceFunction, this, _1, _2));
  }
  // Setup optimization objective
  //
  // If no optimization objective was specified, then default to
  // optimizing path length as computed by the distance() function
  // in the state space.
  if (pdef_)
  {
    if (pdef_->hasOptimizationObjective())
      opt_ = pdef_->getOptimizationObjective();
    else
    {
      OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length "
                  "for the allowed planning time.",
                  getName().c_str());
      opt_.reset(new base::PathLengthOptimizationObjective(si_));
    }
  }
  else
  {
    OMPL_INFORM("%s: problem definition is not set, deferring setup completion...",
                getName().c_str());
    setup_ = false;
  }
}

void ompl::geometric::RRTstarConnect::clear()
{
  Planner::clear();
  sampler_.reset();
  freeMemory();
  if (nnStart_)
    nnStart_->clear();
  if (nnGoal_)
    nnGoal_->clear();

  startMotion_ = NULL;
  goalMotion_  = NULL;

  iterations_     = 0;
  bestCost_       = base::Cost(std::numeric_limits<double>::quiet_NaN());
  bestConnection_ = NULL;
}

bool ompl::geometric::RRTstarConnect::checkMotionAndStartState(const base::State* s1,
                                                               const base::State* s2) const
{
  return si_->isValid(s1) && si_->checkMotion(s1, s2);
}

ompl::base::PlannerStatus
ompl::geometric::RRTstarConnect::solve(const base::PlannerTerminationCondition& ptc)
{
  checkValidity();
  base::GoalSampleableRegion* goal =
    dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

  if (!goal)
  {
    OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
    return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
  }

  bool symCost = opt_->isSymmetric();

  while (const base::State* st = pis_.nextStart())
  {
    Motion* motion = new Motion(si_);
    si_->copyState(motion->state, st);
    motion->cost = opt_->identityCost();
    nnStart_->add(motion);
    startMotion_ = motion;
  }

  if (nnStart_->size() == 0)
  {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }

  if (!goal->couldSample())
  {
    OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
    return base::PlannerStatus::INVALID_GOAL;
  }

  if (!sampler_)
    sampler_ = si_->allocStateSampler();

  OMPL_INFORM("%s: Starting planning with %u start and %u goal states already in datastructure",
              getName().c_str(),
              nnStart_->size(),
              nnGoal_->size());

  if (prune_ && !si_->getStateSpace()->isMetricSpace())
    OMPL_WARN("%s: tree pruning was activated but since the state space %s is not a metric space, "
              "the optimization objective might not satisfy the triangle inequality. You may need "
              "to disable pruning.",
              getName().c_str(),
              si_->getStateSpace()->getName().c_str());

  Motion* solution = bestConnection_;
  if (!solution)
    bestCost_ = opt_->infiniteCost();

  base::Cost bestApproximateCost_ = opt_->infiniteCost();
  Motion* approximation           = NULL;
  bool sufficientlyShort          = false;

  Motion* rmotion     = new Motion(si_);   // Randomly sampled motion
  base::State* rstate = rmotion->state;    // Randomly sampled state in rmotion
  base::State* xstate = si_->allocState(); // Temporary state used in interpolation

  // e+e/d.  K-nearest RRT*
  double k_rrg =
    boost::math::constants::e<double>() +
    (boost::math::constants::e<double>() / (double)si_->getStateSpace()->getDimension());

  std::vector<Motion*> nbh; // Neighbors

  std::vector<base::Cost> costs;              // Cached costs of neighbors
  std::vector<base::Cost> incCosts;           // Cached incremental costs from/to neighbors
  std::vector<std::size_t> sortedCostIndices; // Indices into costs vector

  std::vector<int> valid;           // 1 = nbh[i] is valid, -1 = nbh[i] is invalid
  unsigned int rewireTest      = 0; // Counts the number of checked rewire options.
  unsigned int statesGenerated = 0; // Counts the number of generated states.

  if (solution)
    OMPL_INFORM("%s: Starting planning with existing solution of cost %.5f",
                getName().c_str(),
                solution->cost.value());
  OMPL_INFORM("%s: Initial k-nearest value of %u (start), %u (goal)",
              getName().c_str(),
              (unsigned int)std::ceil(k_rrg * log((double)(nnStart_->size() + 1))),
              (unsigned int)std::ceil(k_rrg * log((double)(nnGoal_->size() + 1))));

  // our functor for sorting nearest neighbors
  CostIndexCompare compareFn(costs, *opt_);

  Motion* nmotionStart = NULL;
  Motion* nmotionGoal  = NULL;
  while (ptc == false)
  {
    iterations_++;

    if (nnGoal_->size() == 0 || pis_.getSampledGoalsCount() < nnGoal_->size() / 2)
    {
      const base::State* st = nnGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
      if (st)
      {
        Motion* motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nnGoal_->add(motion);
      }

      if (nnGoal_->size() == 0)
      {
        OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
        break;
      }
    }

    // sample random state
    sampler_->sampleUniform(rstate);
    if (prune_ && opt_->isCostBetterThan(bestCost_, costToGo(rmotion)))
      continue;

    bool fromStart = true; // true if connected to start tree
    Motion* nmotion;       // Nearest to random state
    if (!alternateTrees_)
    {
      // find closest state in both trees, and pick the closer tree
      // (or the one with lower cost because it has more exploring
      // to do)
      nmotionStart = nnStart_->nearest(rmotion); // Nearest to random state in the start tree
      nmotionGoal  = nnGoal_->nearest(rmotion);  // Nearest to random state in the goal tree
      if (si_->distance(nmotionStart->state, rstate) < si_->distance(nmotionGoal->state, rstate))
      {
        nmotion   = nmotionStart;
        fromStart = true;
      }
      else if (si_->distance(nmotionStart->state, rstate) >
               si_->distance(nmotionGoal->state, rstate))
      {
        nmotion   = nmotionGoal;
        fromStart = false;
      }
      else if (opt_->isCostBetterThan(nmotionStart->cost, nmotionGoal->cost))
      {
        nmotion   = nmotionStart;
        fromStart = true;
      }
      else
      {
        nmotion   = nmotionGoal;
        fromStart = false;
      }
    }
    else
    {
      // find closest state in current tree
      if (nmotionStart != NULL)
      {
        nmotionStart = NULL;
        nmotionGoal  = nnGoal_->nearest(rmotion); // Nearest to random state in the goal tree
        nmotion      = nmotionGoal;
        fromStart    = false;
      }
      else
      {
        nmotionStart = nnStart_->nearest(rmotion); // Nearest to random state in the start tree
        nmotionGoal  = NULL;
        nmotion      = nmotionStart;
        fromStart    = true;
      }
    }

    base::State* dstate = rstate; // Extended state (along line from nearest to random)

    // find state to add to the tree, paying attention to the direction
    double d =
      fromStart ? si_->distance(nmotion->state, rstate) : si_->distance(rstate, nmotion->state);
    if (d > maxDistance_)
    {
      if (fromStart)
        si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
      else
        si_->getStateSpace()->interpolate(rstate, nmotion->state, 1.0 - maxDistance_ / d, xstate);
      dstate = xstate;
    }

    // Check if the motion between the nearest state and the state to add is valid
    if ((fromStart && si_->checkMotion(nmotion->state, dstate)) ||
        (!fromStart && checkMotionAndStartState(dstate, nmotion->state)))
    {
      // create a motion
      Motion* motion = new Motion(si_); // The motion we intend to add to one of the trees
      si_->copyState(motion->state, dstate);
      motion->parent = nmotion;
      if (fromStart)
        motion->incCost = opt_->motionCost(nmotion->state, motion->state);
      else
        motion->incCost = opt_->motionCost(motion->state, nmotion->state);
      motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);

      // Find nearby neighbors of the new motion in the same tree - k-nearest RRT*
      if (fromStart)
      {
        unsigned int k = std::ceil(k_rrg * log((double)(nnStart_->size() + 1)));
        nnStart_->nearestK(motion, k, nbh);
      }
      else
      {
        unsigned int k = std::ceil(k_rrg * log((double)(nnGoal_->size() + 1)));
        nnGoal_->nearestK(motion, k, nbh);
      }
      rewireTest += nbh.size();
      statesGenerated++;

      // cache for distance computations
      //
      // Our cost caches only increase in size, so they're only
      // resized if they can't fit the current neighborhood
      if (costs.size() < nbh.size())
      {
        costs.resize(nbh.size());
        incCosts.resize(nbh.size());
        sortedCostIndices.resize(nbh.size());
      }

      // cache for motion validity (only useful in a symmetric space)
      //
      // Our validity caches only increase in size, so they're
      // only resized if they can't fit the current neighborhood
      if (valid.size() < nbh.size())
        valid.resize(nbh.size());
      std::fill(valid.begin(), valid.begin() + nbh.size(), 0);

      // Finding the nearest neighbor to connect to
      // By default, neighborhood states are sorted by cost, and collision checking
      // is performed in increasing order of cost
      if (delayCC_)
      {
        // calculate all costs and distances
        if (fromStart)
        {
          for (std::size_t i = 0; i < nbh.size(); ++i)
          {
            incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
            costs[i]    = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
          }
        }
        else
        {
          for (std::size_t i = 0; i < nbh.size(); ++i)
          {
            incCosts[i] = opt_->motionCost(motion->state, nbh[i]->state);
            costs[i]    = opt_->combineCosts(incCosts[i], nbh[i]->cost);
          }
        }

        // sort the nodes
        //
        // we're using index-value pairs so that we can get at
        // original, unsorted indices
        for (std::size_t i = 0; i < nbh.size(); ++i)
          sortedCostIndices[i] = i;
        std::sort(sortedCostIndices.begin(), sortedCostIndices.begin() + nbh.size(), compareFn);

        // collision check until a valid motion is found
        //
        // ASYMMETRIC CASE: it's possible that none of these
        // neighbors are valid. This is fine, because motion
        // already has a connection to the tree through
        // nmotion (with populated cost fields!).
        for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
             i != sortedCostIndices.begin() + nbh.size();
             ++i)
        {
          if (nbh[*i] == nmotion ||
              (fromStart && si_->checkMotion(nbh[*i]->state, motion->state)) ||
              (!fromStart && checkMotionAndStartState(motion->state, nbh[*i]->state)))
          {
            motion->incCost = incCosts[*i];
            motion->cost    = costs[*i];
            motion->parent  = nbh[*i];
            valid[*i]       = 1;
            break;
          }
          else
            valid[*i] = -1;
        }
      }
      else // if not delayCC
      {
        if (fromStart)
          motion->incCost = opt_->motionCost(nmotion->state, motion->state);
        else
          motion->incCost = opt_->motionCost(motion->state, nmotion->state);
        motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
        // find which one we connect the new state to
        for (std::size_t i = 0; i < nbh.size(); ++i)
        {
          if (nbh[i] != nmotion)
          {
            if (fromStart)
              incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
            else
              incCosts[i] = opt_->motionCost(motion->state, nbh[i]->state);
            costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
            if (opt_->isCostBetterThan(costs[i], motion->cost))
            {
              if ((fromStart && si_->checkMotion(nbh[i]->state, motion->state)) ||
                  (!fromStart && checkMotionAndStartState(motion->state, nbh[i]->state)))
              {
                motion->incCost = incCosts[i];
                motion->cost    = costs[i];
                motion->parent  = nbh[i];
                valid[i]        = 1;
              }
              else
                valid[i] = -1;
            }
          }
          else
          {
            incCosts[i] = motion->incCost;
            costs[i]    = motion->cost;
            valid[i]    = 1;
          }
        }
      }

      if (prune_)
      {
        if (fromStart && opt_->isCostBetterThan(costToGo(motion, false), bestCost_))
        {
          nnStart_->add(motion);
          motion->parent->children.push_back(motion);
        }
        else if (!fromStart && opt_->isCostBetterThan(costToGo(motion, false), bestCost_))
        {
          nnGoal_->add(motion);
          motion->parent->children.push_back(motion);
        }
        else // If the new motion does not improve the best cost it is ignored.
        {
          --statesGenerated;
          si_->freeState(motion->state);
          delete motion;
          continue;
        }
      }
      else
      {
        // add motion to the tree
        if (fromStart)
          nnStart_->add(motion);
        else
          nnGoal_->add(motion);
        motion->parent->children.push_back(motion);
      }

      // Set to true if a new, shorter connection between both trees is found.
      bool updatedSolution = false;

      // Check for rewiring opportunities
      for (std::size_t i = 0; i < nbh.size(); ++i)
      {
        if (nbh[i] != motion->parent) // If we're not already connected to this one...
        {
          base::Cost nbhIncCost;
          if (symCost)
            nbhIncCost = incCosts[i];
          else if (fromStart)
            nbhIncCost = opt_->motionCost(motion->state, nbh[i]->state);
          else
            nbhIncCost = opt_->motionCost(nbh[i]->state, motion->state);
          base::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);
          if (opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost))
          {
            bool motionValid;
            if (valid[i] == 0)
              if (fromStart)
                motionValid = si_->checkMotion(motion->state, nbh[i]->state);
              else
                motionValid = checkMotionAndStartState(nbh[i]->state, motion->state);
            else
              motionValid = (valid[i] == 1);

            if (motionValid)
            {
              // Remove this node from its parent list
              removeFromParent(nbh[i]);

              // Add this node to the new parent
              nbh[i]->parent  = motion;
              nbh[i]->incCost = nbhIncCost;
              nbh[i]->cost    = nbhNewCost;
              motion->children.push_back(nbh[i]);

              // Update the costs of the node's children.
              // Might find a cheaper connection!
              updatedSolution |= updateChildCosts(nbh[i], (!fromStart));
            }
          }
        }
      }

      // Find nearby neighbors of the new motion in the OTHER tree - k-nearest RRT*Connect
      if (fromStart)
      {
        unsigned int k = std::ceil(k_rrg * log((double)(nnGoal_->size() + 1)));
        nnGoal_->nearestK(motion, k, nbh);
      }
      else
      {
        unsigned int k = std::ceil(k_rrg * log((double)(nnStart_->size() + 1)));
        nnStart_->nearestK(motion, k, nbh);
      }
      rewireTest += nbh.size();

      // We may have to resize our caches again.
      if (costs.size() < nbh.size())
      {
        costs.resize(nbh.size());
        incCosts.resize(nbh.size());
        sortedCostIndices.resize(nbh.size());
      }
      if (valid.size() < nbh.size())
        valid.resize(nbh.size());
      std::fill(valid.begin(), valid.begin() + nbh.size(), 0);

      // Finding the nearest neighbor to connect to
      // By default, neighborhood states are sorted by cost, and collision checking
      // is performed in increasing order of cost
      if (delayCC_)
      {
        // calculate total costs
        if (fromStart)
        {
          for (std::size_t i = 0; i < nbh.size(); ++i)
          {
            incCosts[i] = opt_->motionCost(motion->state, nbh[i]->state);
            costs[i] =
              opt_->combineCosts(opt_->combineCosts(motion->cost, incCosts[i]), nbh[i]->cost);
          }
        }
        else
        {
          for (std::size_t i = 0; i < nbh.size(); ++i)
          {
            incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
            costs[i] =
              opt_->combineCosts(opt_->combineCosts(nbh[i]->cost, incCosts[i]), motion->cost);
          }
        }

        // sort the nodes
        //
        // we're using index-value pairs so that we can get at
        // original, unsorted indices
        for (std::size_t i = 0; i < nbh.size(); ++i)
          sortedCostIndices[i] = i;
        std::sort(sortedCostIndices.begin(), sortedCostIndices.begin() + nbh.size(), compareFn);

        // collision check until a valid motion is found
        //
        // ASYMMETRIC CASE: it's possible that none of these
        // neighbors are valid. This is fine, because motion
        // is already connected to one of the trees (with
        // populated cost fields!).
        for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
             i != sortedCostIndices.begin() + nbh.size();
             ++i)
        {
          if (fromStart && si_->checkMotion(motion->state, nbh[*i]->state))
          {
            valid[*i] = 1;
            if (prune_ && opt_->isCostBetterThan(bestCost_, costs[*i]))
              continue;

            // Check if the neighbor already has a connection, rewire if we are cheaper
            if (nbh[*i]->connection)
            {
              // If we're already connected to it due to rewiring, ignore it.
              if (nbh[*i]->connection->parent == motion)
                continue;
              ompl::base::Cost currentCost =
                opt_->combineCosts(nbh[*i]->connection->cost, nbh[*i]->cost);
              if (!opt_->isCostBetterThan(costs[*i], currentCost))
                continue;
              // OK, we can rewire
              removeFromParent(nbh[*i]->connection);
              nbh[*i]->connection->parent = motion;
              motion->children.push_back(nbh[*i]->connection);
            }
            else
            {
              // Add neighbor to our tree
              Motion* nbhmotion = new Motion(si_);
              si_->copyState(nbhmotion->state, nbh[*i]->state);
              // Add to start tree
              nnStart_->add(nbhmotion);
              nbhmotion->parent = motion;
              motion->children.push_back(nbhmotion);
              // Connect
              nbhmotion->connection = nbh[*i];
              nbh[*i]->connection   = nbhmotion;
            }
            // Update costs
            nbh[*i]->connection->incCost = incCosts[*i];
            nbh[*i]->connection->cost    = costs[*i];
            updatedSolution |= updateChildCosts(nbh[*i]->connection, fromStart);

            if (opt_->isCostBetterThan(costs[*i], bestCost_))
            {
              bestConnection_ = nbh[*i]; // Always the node on the start tree
              bestCost_       = costs[*i];
              updatedSolution = true;
            }
            break;
          }
          else if (!fromStart && checkMotionAndStartState(nbh[*i]->state, motion->state))
          {
            valid[*i] = 1;
            if (prune_ && opt_->isCostBetterThan(bestCost_, costs[*i]))
              continue;

            // Check if the neighbor already has a connection, rewire if we are cheaper
            if (nbh[*i]->connection)
            {
              // If we're already connected to it due to rewiring, ignore it.
              if (nbh[*i]->connection->parent == motion)
                continue;
              ompl::base::Cost currentCost =
                opt_->combineCosts(nbh[*i]->cost, nbh[*i]->connection->cost);
              if (!opt_->isCostBetterThan(costs[*i], currentCost))
                continue;
              // OK, we can rewire
              removeFromParent(nbh[*i]->connection);
              nbh[*i]->connection->parent = motion;
              motion->children.push_back(nbh[*i]->connection);
            }
            else
            {
              // Add neighbor to our tree
              Motion* nbhmotion = new Motion(si_);
              si_->copyState(nbhmotion->state, nbh[*i]->state);
              // Add to goal tree
              nnGoal_->add(nbhmotion);
              nbhmotion->parent = motion;
              motion->children.push_back(nbhmotion);
              // Connect
              nbhmotion->connection = nbh[*i];
              nbh[*i]->connection   = nbhmotion;
            }
            // Update costs
            nbh[*i]->connection->incCost = incCosts[*i];
            nbh[*i]->connection->cost    = costs[*i];
            updatedSolution |= updateChildCosts(nbh[*i]->connection, fromStart);

            if (opt_->isCostBetterThan(costs[*i], bestCost_))
            {
              bestConnection_ = nbh[*i]->connection; // Always the node on the start tree
              bestCost_       = costs[*i];
              updatedSolution = true;
            }
            break;
          }
          else
          {
            valid[*i] = -1;
            if (!solution && fromStart && opt_->isCostBetterThan(costs[*i], bestApproximateCost_))
            {
              bestApproximateCost_ = costs[*i];
              approximation        = nmotion;
            }
          }
        }
      }
      else // if not delayCC
      {
        // find which one we connect the new state to
        for (std::size_t i = 0; i < nbh.size(); ++i)
        {
          if (fromStart)
          {
            incCosts[i] = opt_->motionCost(motion->state, nbh[i]->state);
            costs[i] =
              opt_->combineCosts(opt_->combineCosts(motion->cost, incCosts[i]), nbh[i]->cost);
          }
          else
          {
            incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
            costs[i] =
              opt_->combineCosts(opt_->combineCosts(nbh[i]->cost, incCosts[i]), motion->cost);
          }

          if (fromStart && si_->checkMotion(motion->state, nbh[i]->state))
          {
            valid[i] = 1;
            if (prune_ && opt_->isCostBetterThan(bestCost_, costs[i]))
              continue;

            // Check if the neighbor already has a connection, rewire if we are cheaper
            if (nbh[i]->connection)
            {
              // If we're already connected to it due to rewiring, ignore it.
              if (nbh[i]->connection->parent == motion)
                continue;
              ompl::base::Cost currentCost =
                opt_->combineCosts(nbh[i]->connection->cost, nbh[i]->cost);
              if (!opt_->isCostBetterThan(costs[i], currentCost))
                continue;
              // OK, we can rewire
              removeFromParent(nbh[i]->connection);
              nbh[i]->connection->parent = motion;
              motion->children.push_back(nbh[i]->connection);
            }
            else
            {
              // Add neighbor to our tree
              Motion* nbhmotion = new Motion(si_);
              si_->copyState(nbhmotion->state, nbh[i]->state);
              // Add to start tree
              nnStart_->add(nbhmotion);
              nbhmotion->parent = motion;
              motion->children.push_back(nbhmotion);
              // Connect
              nbhmotion->connection = nbh[i];
              nbh[i]->connection    = nbhmotion;
            }
            // Update costs
            nbh[i]->connection->incCost = incCosts[i];
            nbh[i]->connection->cost    = costs[i];
            updatedSolution |= updateChildCosts(nbh[i]->connection, fromStart);

            if (opt_->isCostBetterThan(costs[i], bestCost_))
            {
              bestConnection_ = nbh[i]; // Always the node on the start tree
              bestCost_       = costs[i];
              updatedSolution = true;
            }
          }
          else if (!fromStart && checkMotionAndStartState(nbh[i]->state, motion->state))
          {
            valid[i] = 1;
            if (prune_ && opt_->isCostBetterThan(bestCost_, costs[i]))
              continue;

            // Check if the neighbor already has a connection, rewire if we are cheaper
            if (nbh[i]->connection)
            {
              // If we're already connected to it due to rewiring, ignore it.
              if (nbh[i]->connection->parent == motion)
                continue;
              ompl::base::Cost currentCost =
                opt_->combineCosts(nbh[i]->cost, nbh[i]->connection->cost);
              if (!opt_->isCostBetterThan(costs[i], currentCost))
                continue;
              // OK, we can rewire
              removeFromParent(nbh[i]->connection);
              nbh[i]->connection->parent = motion;
              motion->children.push_back(nbh[i]->connection);
            }
            else
            {
              // Add neighbor to our tree
              Motion* nbhmotion = new Motion(si_);
              si_->copyState(nbhmotion->state, nbh[i]->state);
              // Add to goal tree
              nnGoal_->add(nbhmotion);
              nbhmotion->parent = motion;
              motion->children.push_back(nbhmotion);
              // Connect
              nbhmotion->connection = nbh[i];
              nbh[i]->connection    = nbhmotion;
            }
            // Update costs
            nbh[i]->connection->incCost = incCosts[i];
            nbh[i]->connection->cost    = costs[i];
            updatedSolution |= updateChildCosts(nbh[i]->connection, fromStart);

            if (opt_->isCostBetterThan(costs[i], bestCost_))
            {
              bestConnection_ = nbh[i]->connection; // Always the node on the start tree
              bestCost_       = costs[i];
              updatedSolution = true;
            }
          }
          else
          {
            valid[i] = -1;
            if (!solution && fromStart && opt_->isCostBetterThan(costs[i], bestApproximateCost_))
            {
              bestApproximateCost_ = costs[i];
              approximation        = nmotion;
            }
          }
        }
      }

      // Checking for solution or iterative improvement
      if (updatedSolution)
      {
        solution          = bestConnection_;
        sufficientlyShort = opt_->isSatisfied(bestCost_);
        if (sufficientlyShort)
        {
          break;
        }

        if (prune_)
        {
          int n = pruneTree(bestCost_);
          statesGenerated -= n;
        }
      }
    }

    // terminate if a sufficient solution is found
    if (solution && sufficientlyShort)
      break;
  }

  bool approximate   = (solution == NULL);
  bool addedSolution = false;
  if (approximate)
    solution = approximation;

  if (solution != NULL)
  {
    ptc.terminate();
    // construct the solution path
    std::deque<Motion*> mpath;
    Motion* solutionStart = solution;
    while (solutionStart != NULL)
    {
      mpath.push_back(solutionStart);
      solutionStart = solutionStart->parent;
    }
    Motion* solutionGoal = solution->connection;
    bool skip_first      = true;
    while (solutionGoal != NULL)
    {
      // Skip the first node since it is identical to the last node on the start tree side
      if (skip_first)
        skip_first = false;
      else
        mpath.push_front(solutionGoal);
      solutionGoal = solutionGoal->parent;
    }

    // set the solution path
    PathGeometric* geoPath = new PathGeometric(si_);
    for (int i = mpath.size() - 1; i >= 0; --i)
      geoPath->append(mpath[i]->state);
    unsigned int* startIndex = 0;
    if (!pdef_->hasStartState(geoPath->getStates().front(), startIndex))
    {
      geoPath->reverse();
    }
    base::PathPtr path(geoPath);
    // Add the solution path.
    base::PlannerSolution psol(path);
    psol.setPlannerName(getName());
    if (approximate)
      psol.setApproximate(bestApproximateCost_.value());
    // Does the solution satisfy the optimization objective?
    // psol.setOptimized(opt_, bestCost, sufficientlyShort);
    pdef_->addSolutionPath(psol);

    addedSolution = true;
  }

  si_->freeState(xstate);
  if (rmotion->state)
    si_->freeState(rmotion->state);
  delete rmotion;

  OMPL_INFORM("%s: Created %u new states. Checked %u rewire options.",
              getName().c_str(),
              statesGenerated,
              rewireTest);
  OMPL_INFORM(
    "%s: addedSolution = %i, approximate = %i.", getName().c_str(), addedSolution, approximate);

  return base::PlannerStatus(addedSolution, approximate);
}

void ompl::geometric::RRTstarConnect::removeFromParent(Motion* m)
{
  for (std::vector<Motion*>::iterator it = m->parent->children.begin();
       it != m->parent->children.end();
       ++it)
    if (*it == m)
    {
      m->parent->children.erase(it);
      break;
    }
}

bool ompl::geometric::RRTstarConnect::updateChildCosts(Motion* m, bool fromStart)
{
  bool updatedSolution = false;
  if (m->connection)
  {
    ompl::base::Cost cost = opt_->combineCosts(m->cost, m->connection->cost);
    if (opt_->isCostBetterThan(cost, bestCost_))
    {
      if (fromStart)
        bestConnection_ = m;
      else
        bestConnection_ = m->connection;
      bestCost_       = cost;
      updatedSolution = true;
    }
  }

  for (std::size_t i = 0; i < m->children.size(); ++i)
  {
    m->children[i]->cost = opt_->combineCosts(m->cost, m->children[i]->incCost);
    updatedSolution |= updateChildCosts(m->children[i], fromStart);
  }

  return updatedSolution;
}

void ompl::geometric::RRTstarConnect::freeMemory()
{
  if (nnStart_)
  {
    std::vector<Motion*> motions;
    nnStart_->list(motions);
    for (std::size_t i = 0; i < motions.size(); ++i)
    {
      if (motions[i]->state)
        si_->freeState(motions[i]->state);
      delete motions[i];
    }
  }
  if (nnGoal_)
  {
    std::vector<Motion*> motions;
    nnGoal_->list(motions);
    for (std::size_t i = 0; i < motions.size(); ++i)
    {
      if (motions[i]->state)
        si_->freeState(motions[i]->state);
      delete motions[i];
    }
  }
}

void ompl::geometric::RRTstarConnect::getPlannerData(base::PlannerData& data) const
{
  Planner::getPlannerData(data);

  std::vector<Motion*> startMotions;
  if (nnStart_)
    nnStart_->list(startMotions);

  for (std::size_t i = 0; i < startMotions.size(); ++i)
  {
    if (startMotions[i]->parent == NULL)
      data.addStartVertex(base::PlannerDataVertex(startMotions[i]->state, START_TREE_VERTEX));
    else
      data.addEdge(base::PlannerDataVertex(startMotions[i]->parent->state, START_TREE_VERTEX),
                   base::PlannerDataVertex(startMotions[i]->state, START_TREE_VERTEX));
  }

  std::vector<Motion*> goalMotions;
  if (nnGoal_)
    nnGoal_->list(goalMotions);
  for (std::size_t i = 0; i < goalMotions.size(); ++i)
  {
    if (goalMotions[i]->parent == NULL)
      data.addGoalVertex(base::PlannerDataVertex(goalMotions[i]->state, GOAL_TREE_VERTEX));
    else
      data.addEdge(base::PlannerDataVertex(goalMotions[i]->parent->state, GOAL_TREE_VERTEX),
                   base::PlannerDataVertex(goalMotions[i]->state, GOAL_TREE_VERTEX));
    // Add the connect edges.
    if (goalMotions[i]->connection)
      data.addEdge(base::PlannerDataVertex(goalMotions[i]->connection->state, START_TREE_VERTEX),
                   base::PlannerDataVertex(goalMotions[i]->state, GOAL_TREE_VERTEX));
  }
}

int ompl::geometric::RRTstarConnect::pruneTree(const TreeData& nn, const base::Cost pruneTreeCost)
{
  const int tree_size = nn->size();
  pruneScratchSpace_.newTree.reserve(tree_size);
  pruneScratchSpace_.newTree.clear();
  pruneScratchSpace_.toBePruned.reserve(tree_size);
  pruneScratchSpace_.toBePruned.clear();
  pruneScratchSpace_.candidates.clear();
  pruneScratchSpace_.candidates.push_back(startMotion_);
  std::size_t j = 0;
  while (j != pruneScratchSpace_.candidates.size())
  {
    Motion* candidate = pruneScratchSpace_.candidates[j++];
    if (opt_->isCostBetterThan(pruneTreeCost, costToGo(candidate)))
      pruneScratchSpace_.toBePruned.push_back(candidate);
    else
    {
      pruneScratchSpace_.newTree.push_back(candidate);
      pruneScratchSpace_.candidates.insert(pruneScratchSpace_.candidates.end(),
                                           candidate->children.begin(),
                                           candidate->children.end());
    }
  }

  // To create the new nn takes one order of magnitude in time more than just checking how many
  // states would be pruned. Therefore, only prune if it removes a significant amount of states.
  if ((double)pruneScratchSpace_.newTree.size() / tree_size < pruneStatesThreshold_)
  {
    for (std::size_t i = 0; i < pruneScratchSpace_.toBePruned.size(); ++i)
      deleteBranch(pruneScratchSpace_.toBePruned[i]);

    nn->clear();
    nn->add(pruneScratchSpace_.newTree);

    return (tree_size - pruneScratchSpace_.newTree.size());
  }
  return 0;
}

int ompl::geometric::RRTstarConnect::pruneTree(const base::Cost pruneTreeCost)
{
  return pruneTree(nnStart_, pruneTreeCost) + pruneTree(nnGoal_, pruneTreeCost);
}

void ompl::geometric::RRTstarConnect::deleteBranch(Motion* motion)
{
  removeFromParent(motion);
  if (motion->connection)
  {
    motion->connection->connection = NULL;
    motion->connection             = NULL;
  }
  std::vector<Motion*>& toDelete = pruneScratchSpace_.candidates;
  toDelete.clear();
  toDelete.push_back(motion);

  while (!toDelete.empty())
  {
    Motion* mto_delete = toDelete.back();
    toDelete.pop_back();

    for (std::size_t i = 0; i < mto_delete->children.size(); ++i)
      toDelete.push_back(mto_delete->children[i]);

    si_->freeState(mto_delete->state);
    delete mto_delete;
  }
}

ompl::base::Cost ompl::geometric::RRTstarConnect::costToGo(const Motion* motion,
                                                           const bool shortest) const
{
  base::Cost costToCome;
  if (shortest)
    costToCome = opt_->motionCost(startMotion_->state, motion->state); // h_s
  else
    costToCome = motion->cost; // d_s

  const base::Cost costToGo =
    base::goalRegionCostToGo(motion->state, pdef_->getGoal().get()); // h_g
  return opt_->combineCosts(costToCome, costToGo);                   // h_s + h_g
}
