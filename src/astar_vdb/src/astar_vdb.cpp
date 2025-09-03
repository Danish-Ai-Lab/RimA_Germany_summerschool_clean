
#include <astar_vdb/astar_vdb.h>
#include <eigen3/Eigen/Geometry>

#include <openvdb/tools/Morphology.h>


namespace astar_vdb {

AstarVDB::AstarVDB(typename ValueGridT::Ptr value_grid,
                   bool debug_mode,
                   int ground_seach_level,
                   bool ground_cache_on,
                   int successor_variant,
                   int successor_restriction,
                   int stepheight,
                   int dilate,
                   int slope_ground_range,
                   float max_slope_angle_degrees)
  : PlannerVDBBase(debug_mode,
                   ground_seach_level,
                   ground_cache_on,
                   successor_variant,
                   successor_restriction,
                   stepheight,
                   dilate,
                   slope_ground_range,
                   max_slope_angle_degrees)
{
  m_open_list = std::make_shared<PriorityQueue>();

  // setup the grid where dstar info will be stored
  if (!AStarGridT::isRegistered())
  {
    AStarGridT::registerGrid();
  }
  m_astar_grid = AStarGridT::create(AStarDataNode());
  m_astar_grid->setTransform(value_grid->transformPtr());
  m_astar_grid->setGridClass(openvdb::GRID_LEVEL_SET);
  m_astar_grid_acc = std::make_shared<AStarGridT::Accessor>(m_astar_grid->getAccessor());

  if (m_debug_mode)
  {
    m_expand_grid = openvdb::Int32Grid::create();
    m_expand_grid->setTransform(value_grid->transformPtr());
    m_expand_grid->setGridClass(openvdb::GRID_LEVEL_SET);
    m_expand_grid_acc =
      std::make_shared<openvdb::Int32Grid::Accessor>(m_expand_grid->getAccessor());
  }

  initPlanner(value_grid);

  m_open_list = std::make_shared<PriorityQueue>();
}


void AstarVDB::initialize()
{
  if (m_debug_mode)
  {
    m_expand_grid->clear();
  }

  // Initialize the open list
  m_open_list->clear();

  // Initialize the closed list
  m_astar_grid->clear();

  // put the starting node on the open
  // list (you can leave its f at zero)
  auto data      = m_astar_grid_acc->getValue(m_search_start).getData();
  data.f         = 0;
  data.g         = 0;
  data.h         = 0;
  data.parent    = m_search_start;
  data.is_closed = false;
  data.is_open   = true;
  m_astar_grid_acc->setValue(m_search_start, data);
  m_open_list->push(m_search_start, data.f);
}

bool AstarVDB::setStartAndGoal(const openvdb::Coord& start, const openvdb::Coord& goal)
{
  m_search_start = start;
  m_search_goal  = goal;

  std::cout << std::endl;
  std::cout << "Start-End node generation:" << std::endl;
  std::cout << "--------------------------" << std::endl;

  if (!generateValidNode(m_search_start, m_cspace_grid))
  {
    // std::cout << "Error: No valid start position found!" << std::endl;
    // m_state = planner_vdb_base::SearchState::FAILURE;
    // return false;
    std::cout << "Could not generate valid node from goal" << std::endl;
    std::cout << "Trying with input node" << std::endl;
  }

  if (!generateValidNode(m_search_goal, m_cspace_grid))
  {
    // std::cout << "Error: No valid goal position found!" << std::endl;
    // m_state = planner_vdb_base::SearchState::FAILURE;
    // return false;
    std::cout << "Could not generate valid node from start" << std::endl;
    std::cout << "Trying with input node" << std::endl;
  }

  m_start_index_coord = m_search_start;
  m_goal_index_coord  = m_search_goal;

  std::cout << "Start: " << m_start_index_coord << std::endl;
  std::cout << "Goal: " << m_goal_index_coord << std::endl;

  initialize();
  return true;
}

bool AstarVDB::isPathAvailable()
{
  return m_astar_grid_acc->getValue(m_search_goal).getData().g <
         std::numeric_limits<double>::infinity();
}


std::vector<openvdb::Coord> AstarVDB::extractPath()
{
  std::cout << std::endl;
  std::cout << "Path extraction:" << std::endl;
  std::cout << "----------------" << std::endl;

  std::vector<openvdb::Coord> path;

  if (m_state == planner_vdb_base::SearchState::SUCCESS)
  {
    std::cout << "Search found goal state" << std::endl;
    auto current = m_search_goal;
    path.push_back(m_search_goal);
    double cost = 0; // m_astar_grid_acc->getValue(current).getData().g;


    std::cout << "Displaying Solution" << std::endl;
    int steps = 0;
    for (;;)
    {
      // std::cout << "current: " << current << std::endl;
      auto data = m_astar_grid_acc->getValue(current).getData();
      // cost += getCost(current, data.parent, m_value_grid_acc, m_cspace_grid_acc, true);
      if (current == m_search_start)
      {
        // cost = data.h;
        break;
      }

      current = data.parent;

      path.push_back(current);
      steps++;
    }

    // Path has to be reversed so that is is from start to goal
    std::reverse(path.begin(), path.end());

    // path is true start->goal
    cost = 0;
    for (size_t i = 0; i < path.size() - 1; ++i)
    {
      cost += getCost(path[i], path[i + 1], m_value_grid_acc, m_cspace_grid_acc, true);
    }

    std::cout << "Solution steps: " << steps << std::endl;
    std::cout << "Total costs: " << cost << std::endl;
    std::cout << "Extracting path done. " << path[0] << " to " << path[path.size() - 1]
              << std::endl;
  }


  return path;
}


planner_vdb_base::SearchState AstarVDB::step()
{
  if ((m_state == planner_vdb_base::SearchState::SUCCESS) ||
      (m_state == planner_vdb_base::SearchState::FAILURE))
  {
    std::cout << "step: already failed or succeeded" << std::endl;
    return m_state;
  }

  if (m_cancel)
  {
    std::cout << "step: cancel request" << std::endl;
    initialize();
    m_cancel = false;
    m_state  = planner_vdb_base::SearchState::FAILURE;
    return m_state;
  }


  if (m_open_list->empty())
  {
    std::cout << "planning failed: empty queue" << std::endl;
    m_state = planner_vdb_base::SearchState::FAILURE;
    return m_state;
  }

  // do search step

  // Remove this vertex from the open list
  std::pair<openvdb::Coord, double> q = m_open_list->pop();
  node_expand_count++;

  auto q_data = m_astar_grid_acc->getValue(q.first).getData();
  // std::cout << "f: " << q_data.f << std::endl;

  // Add this vertex to the closed list
  q_data.is_closed = true;
  q_data.is_open   = false;
  m_astar_grid_acc->setValue(q.first, q_data);

  // check if we reached target
  if (q.first == m_search_goal)
  {
    m_state = planner_vdb_base::SearchState::SUCCESS;
    return m_state;
  }


  //  Generating all the successors
  auto succ = getSuccessorCoords(q.first);

  for (const openvdb::Coord& s : succ)
  {
    auto succ_data = m_astar_grid_acc->getValue(s).getData();
    if (succ_data.is_closed)
    {
      continue;
    }

    if (m_debug_mode)
    {
      auto expanded_grid_value = m_expand_grid_acc->getValue(s);
      expanded_grid_value += 1;
      m_expand_grid_acc->setValue(s, expanded_grid_value);
    }

    // set their parents to q
    succ_data.parent = q.first;

    // else, compute both g and h for successor
    // double old_f = succ_data.f;
    double old_g = succ_data.g;

    // succ_data.g = q_data.g + getCost(q.first, s, m_value_grid_acc, m_cspace_grid_acc, true);
    // reverse because search from goal to start
    succ_data.g = q_data.g + getCost(s, q.first, m_value_grid_acc, m_cspace_grid_acc, true);
    succ_data.h = getHeuristic(s, m_search_start, m_search_goal);
    succ_data.f = succ_data.g + succ_data.h;


    // double f;
    // if (m_open_list->contains(s, f))
    if (succ_data.is_open)
    {
      //  If it is on the open list already, check to see if this path to that square is better,
      //  using G cost as the measure. A lower G cost means that this is a better path. If so,
      //  change the parent of the square to the current square, and recalculate the G and F scores
      //  of the square. If you are keeping your open list sorted by F score, you may need to resort
      //  the list to account for the change.
      if (old_g > succ_data.g)
      {
        m_astar_grid_acc->setValue(s, succ_data);
        m_open_list->update(s, succ_data.f);
      }
    }
    else
    {
      //  If it isnt on the open list, add it to the open list.
      //  Make the current square the parent of this square.
      //  Record the F, G, and H costs of the square.
      succ_data.is_open = true;
      m_astar_grid_acc->setValue(s, succ_data);
      m_open_list->push(s, succ_data.f);
    }
  }


  m_state = planner_vdb_base::SearchState::SEARCHING;
  return m_state;
}


void AstarVDB::printSearchStats()
{
  std::cout << "Parent nodes expanded: " << node_expand_count << std::endl;
  if (m_debug_mode)
  {
    int operations_per_node_sum = 0;
    int nodes_expanded          = 0;
    for (openvdb::Int32Grid::ValueOnCIter iter = m_expand_grid->cbeginValueOn(); iter; ++iter)
    {
      nodes_expanded += 1;
      operations_per_node_sum += m_expand_grid_acc->getValue(iter.getCoord());
    }
    std::cout << "Nodes expanded: " << nodes_expanded << std::endl;

    std::cout << "Operations per node sum: " << operations_per_node_sum << std::endl;
    std::cout << "Get successor count: " << get_succ_count << std::endl;
    std::cout << "Get cost count: " << get_cost_count << std::endl;
    std::cout << "Get heuristic count: " << get_heuristic_count << std::endl;

    std::cout << "Heap size: " << m_open_list->size() << std::endl;
    std::cout << "Ground cache hit: " << (double)chache_hit_count / (double)has_ground_count * 100
              << "% (" << chache_hit_count << " / " << has_ground_count << ")" << std::endl;

    std::cout << m_open_list->getStats() << std::endl;
  }
}


} // namespace astar_vdb
