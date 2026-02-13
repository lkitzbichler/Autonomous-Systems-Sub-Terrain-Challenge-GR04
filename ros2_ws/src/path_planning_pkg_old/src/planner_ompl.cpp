#include "path_planning_pkg/path_planner_node.hpp"

#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <algorithm>
#include <cmath>

bool PathPlannerNode::planPathOmpl(const Eigen::Vector3d &start,
                                   const Eigen::Vector3d &goal,
                                   std::vector<Eigen::Vector3d> *path_out) {
  // --- OMPL planning ---
  // Plan a collision-free path using OMPL RRT* in 3D.
  path_out->clear();

  std::shared_ptr<octomap::OcTree> map;
  octomap::point3d min;
  octomap::point3d max;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = map_;
    min = map_min_;
    max = map_max_;
  }
  if (!map) {
    return false;
  }

  octomap::OcTreeKey start_key;
  if (!map->coordToKeyChecked(
          octomap::point3d(start.x(), start.y(), start.z()), start_key)) {
    return false;
  }
  if (!isFreeKeyInMap(map.get(), start_key, min, max, clearance_radius_)) {
    if (!findNearestFreeKey(start_key, &start_key)) {
      return false;
    }
  }
  const octomap::point3d start_coord = map->keyToCoord(start_key);

  octomap::OcTreeKey goal_key;
  if (!map->coordToKeyChecked(
          octomap::point3d(goal.x(), goal.y(), goal.z()), goal_key)) {
    return false;
  }
  if (!isFreeKeyInMap(map.get(), goal_key, min, max, clearance_radius_)) {
    if (!findNearestFreeKey(goal_key, &goal_key)) {
      return false;
    }
  }
  const octomap::point3d goal_coord = map->keyToCoord(goal_key);

  // OMPL setup: 3D real vector space with Octomap-based state validity.
  auto space = std::make_shared<ompl::base::RealVectorStateSpace>(3);
  ompl::base::RealVectorBounds bounds(3);
  bounds.setLow(0, min.x());
  bounds.setLow(1, min.y());
  bounds.setLow(2, min.z());
  bounds.setHigh(0, max.x());
  bounds.setHigh(1, max.y());
  bounds.setHigh(2, max.z());
  space->setBounds(bounds);

  ompl::geometric::SimpleSetup ss(space);
  ss.setStateValidityChecker([this, map, min, max](const ompl::base::State *state) {
    const auto *rv = state->as<ompl::base::RealVectorStateSpace::StateType>();
    const octomap::point3d p(rv->values[0], rv->values[1], rv->values[2]);
    if (!this->coordInBounds(p, min, max)) {
      return false;
    }
    auto *node = map->search(p);
    if (!node) {
      return ompl_allow_unknown_;
    }
    if (map->isNodeOccupied(node)) {
      return false;
    }
    return this->isFreeCoordInMap(map.get(), p, min, max, clearance_radius_);
  });

  ss.getSpaceInformation()->setStateValidityCheckingResolution(ompl_resolution_);

  ompl::base::ScopedState<> start_state(space);
  start_state[0] = start_coord.x();
  start_state[1] = start_coord.y();
  start_state[2] = start_coord.z();
  ompl::base::ScopedState<> goal_state(space);
  goal_state[0] = goal_coord.x();
  goal_state[1] = goal_coord.y();
  goal_state[2] = goal_coord.z();

  ss.setStartAndGoalStates(start_state, goal_state);
  auto planner = std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation());
  planner->setRange(ompl_range_);
  planner->setGoalBias(ompl_goal_bias_);
  ss.setPlanner(planner);

  if (!ss.solve(ompl_planning_time_)) {
    return false;
  }

  if (ompl_simplify_) {
    ompl::geometric::PathSimplifier simplifier(ss.getSpaceInformation());
    simplifier.shortcutPath(ss.getSolutionPath());
    simplifier.smoothBSpline(ss.getSolutionPath());
  }

  const auto &geo_path = ss.getSolutionPath().getStates();
  path_out->reserve(geo_path.size());
  for (const auto *s : geo_path) {
    const auto *rv = s->as<ompl::base::RealVectorStateSpace::StateType>();
    path_out->emplace_back(rv->values[0], rv->values[1], rv->values[2]);
  }
  return path_out->size() >= 2;
}

std::vector<Eigen::Vector3d> PathPlannerNode::simplifyPointPath(
    const std::vector<Eigen::Vector3d> &path) const {
  // --- Path post-processing ---
  if (path.size() < 2) {
    return path;
  }

  // Distance-based downsampling followed by optional line-of-sight pruning.
  std::vector<Eigen::Vector3d> simplified;
  simplified.push_back(path.front());
  double accum = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    accum += (path[i] - simplified.back()).norm();
    if (accum >= path_simplify_distance_ || i + 1 == path.size()) {
      simplified.push_back(path[i]);
      accum = 0.0;
    }
  }

  if (simplified.size() < 2 && path.size() >= 2) {
    simplified.push_back(path.back());
  }

  if (use_line_of_sight_prune_) {
    return prunePathLineOfSight(simplified);
  }

  return simplified;
}

bool PathPlannerNode::isSegmentFree(const Eigen::Vector3d &start,
                                    const Eigen::Vector3d &goal) const {
  std::shared_ptr<octomap::OcTree> map;
  octomap::point3d min;
  octomap::point3d max;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = map_;
    min = map_min_;
    max = map_max_;
  }
  if (!map) {
    return false;
  }

  const Eigen::Vector3d delta = goal - start;
  const double length = delta.norm();
  if (length <= 1e-6) {
    return true;
  }

  const double step = std::max(0.05, line_of_sight_step_);
  const int steps = static_cast<int>(std::ceil(length / step));
  const Eigen::Vector3d dir = delta / length;

  for (int i = 0; i <= steps; ++i) {
    const Eigen::Vector3d p = start + dir * (static_cast<double>(i) * step);
    const octomap::point3d coord(p.x(), p.y(), p.z());
    if (!isFreeCoordInMap(map.get(), coord, min, max, clearance_radius_)) {
      return false;
    }
  }

  return true;
}

std::vector<Eigen::Vector3d> PathPlannerNode::prunePathLineOfSight(
    const std::vector<Eigen::Vector3d> &path) const {
  if (path.size() <= 2) {
    return path;
  }

  std::vector<Eigen::Vector3d> pruned;
  pruned.reserve(path.size());
  size_t i = 0;
  pruned.push_back(path.front());

  while (i + 1 < path.size()) {
    size_t best = i + 1;
    for (size_t j = path.size() - 1; j > i; --j) {
      if (isSegmentFree(path[i], path[j])) {
        best = j;
        break;
      }
    }
    pruned.push_back(path[best]);
    i = best;
  }

  return pruned;
}
