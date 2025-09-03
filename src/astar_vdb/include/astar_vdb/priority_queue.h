#pragma once

#include "astar_utils.h"
#include <algorithm>
#include <cassert>
#include <openvdb/openvdb.h>
#include <sstream>
#include <string>

#define DEBUG 0
namespace astar_vdb {


class PriorityCompare
{
public:
  bool operator()(const std::pair<openvdb::Coord, double>& a,
                  const std::pair<openvdb::Coord, double>& b) const
  {
    return a.second < b.second;
  }
};

class PriorityQueue
{
private:
  PriorityCompare priority_comparator;
  std::multiset<std::pair<openvdb::Coord, double>, PriorityCompare> c;


  int push_count     = 0;
  int pop_count      = 0;
  int top_count      = 0;
  int contains_count = 0;
  int remove_count   = 0;
  int update_count   = 0;

public:
  PriorityQueue() {}

  void resetStats()
  {
    push_count     = 0;
    pop_count      = 0;
    top_count      = 0;
    contains_count = 0;
    remove_count   = 0;
    update_count   = 0;
  }

  bool empty() { return c.empty(); }

  std::size_t size() { return c.size(); }

  void clear()
  {
    c.clear();
    resetStats();
  }

  void push(openvdb::Coord coord, double f) // O(log n)
  {
#if DEBUG
    push_count++;
#endif
    c.insert(std::make_pair(coord, f));
  }

  std::pair<openvdb::Coord, double> pop()
  {
    assert(!c.empty());
    pop_count++;

    std::pair<openvdb::Coord, double> result = *c.begin();
    c.erase(c.begin());
    return result;
  }

  //   double topdouble() // O(1)
  //   {
  // #if DEBUG
  //     assert(!c.empty());
  //     top_count++;
  // #endif

  //     return c.begin()->second;
  //   }

  std::pair<openvdb::Coord, double> top() // O(1)
  {
#if DEBUG
    assert(!c.empty());
    top_count++;
#endif

    return *c.begin();
  }

  bool contains(openvdb::Coord coord, double& f) // O(n)
  {
#if DEBUG
    contains_count++;
#endif
    for (auto item : c)
    { // TODO: std::find_if?
      if (item.first == coord)
      {
        f = item.second;
        return true;
      }
    }
    return false;
  }


  bool remove(const openvdb::Coord& coord) // O(n)
  {
#if DEBUG
    remove_count++;
#endif
    bool found = false;
    for (auto it = c.begin(); it != c.end();)
    {
      if (it->first == coord)
      {
        it    = c.erase(it);
        found = true;
        break;
      }
      else
      {
        ++it;
      }
    }
    return found;
  }


  bool update(const openvdb::Coord& coord, double new_f) // O(n)
  {
#if DEBUG
    update_count++;
#endif

    // TODO: Better way?
    bool found = remove(coord); // O(n)
    remove_count--;
    if (found)
    {
      push(coord, new_f); // O(log n)
      push_count--;
    }
    // bool found = false;

    // for (auto it = c.begin(); it != c.end();)
    // {
    //   if (it->first == coord)
    //   {
    //     Set::iterator hint = it;
    //     it->second = new_double;
    //     found      = true;
    //     break;
    //   }
    //   else
    //   {
    //     ++it;
    //   }
    // }

    return found;
  }

  void printContent()
  {
    std::cout << "Heap Content:" << std::endl;
    if (empty())
    {
      return;
    }

    for (auto it = c.begin(); it != c.end(); ++it)
    {
      std::cout << it->first << ", " << it->second << std::endl;
    }
  }

  std::string getStats()
  {
#if DEBUG
    std::ostringstream oss;
    oss << "push_count = " << push_count << "\n"
        << "pop_count = " << pop_count << "\n"
        << "top_count = " << top_count << "\n"
        << "contains_count = " << contains_count << "\n"
        << "remove_count = " << remove_count << "\n"
        << "update_count = " << update_count;

    return oss.str();
#else
    return "Priority queue stats only available if DEBUG macro set.";
#endif
  }
};
} // namespace astar_vdb