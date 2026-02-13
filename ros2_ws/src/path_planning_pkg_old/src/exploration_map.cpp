#include "path_planning_pkg/path_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_set>

bool PathPlannerNode::coordInBounds(const octomap::point3d &coord,
                                    const octomap::point3d &min,
                                    const octomap::point3d &max) {
  return coord.x() >= min.x() && coord.x() <= max.x() &&
         coord.y() >= min.y() && coord.y() <= max.y() &&
         coord.z() >= min.z() && coord.z() <= max.z();
}

bool PathPlannerNode::isFreeCoordInMap(const octomap::OcTree *tree,
                                       const octomap::point3d &coord,
                                       const octomap::point3d &min,
                                       const octomap::point3d &max,
                                       double clearance_radius) {
  if (!tree) {
    return false;
  }
  if (!coordInBounds(coord, min, max)) {
    return false;
  }

  auto *node = tree->search(coord);
  if (!node || tree->isNodeOccupied(node)) {
    return false;
  }

  if (clearance_radius <= 1e-6) {
    return true;
  }

  // Enforce a spherical clearance by checking neighboring voxels.
  const double res = tree->getResolution();
  const int steps = static_cast<int>(std::ceil(clearance_radius / res));
  for (int dx = -steps; dx <= steps; ++dx) {
    for (int dy = -steps; dy <= steps; ++dy) {
      for (int dz = -steps; dz <= steps; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        octomap::point3d c(coord.x() + dx * res, coord.y() + dy * res,
                           coord.z() + dz * res);
        if (!coordInBounds(c, min, max)) {
          return false;
        }
        auto *n = tree->search(c);
        if (n && tree->isNodeOccupied(n)) {
          return false;
        }
      }
    }
  }
  return true;
}

bool PathPlannerNode::isFreeKeyInMap(const octomap::OcTree *tree,
                                     const octomap::OcTreeKey &key,
                                     const octomap::point3d &min,
                                     const octomap::point3d &max,
                                     double clearance_radius) {
  if (!tree) {
    return false;
  }
  const octomap::point3d coord = tree->keyToCoord(key);
  return isFreeCoordInMap(tree, coord, min, max, clearance_radius);
}

bool PathPlannerNode::isFrontierKeyInMap(const octomap::OcTree *tree,
                                         const octomap::OcTreeKey &key,
                                         const octomap::point3d &min,
                                         const octomap::point3d &max) {
  if (!tree) {
    return false;
  }
  const double res = tree->getResolution();
  const octomap::point3d coord = tree->keyToCoord(key);
  const int dirs[6][3] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                          {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};
  for (const auto &d : dirs) {
    octomap::point3d ncoord(coord.x() + d[0] * res, coord.y() + d[1] * res,
                            coord.z() + d[2] * res);
    if (!coordInBounds(ncoord, min, max)) {
      return true;
    }
    auto *n = tree->search(ncoord);
    if (!n) {
      // Unknown neighbor => frontier.
      return true;
    }
  }
  return false;
}

std::size_t KeyHash::operator()(const octomap::OcTreeKey &key) const noexcept {
  const uint64_t packed =
      (static_cast<uint64_t>(key.k[0]) << 32) |
      (static_cast<uint64_t>(key.k[1]) << 16) |
      static_cast<uint64_t>(key.k[2]);
  return std::hash<uint64_t>{}(packed);
}

bool KeyEq::operator()(const octomap::OcTreeKey &a,
                       const octomap::OcTreeKey &b) const noexcept {
  return a.k[0] == b.k[0] && a.k[1] == b.k[1] && a.k[2] == b.k[2];
}

bool PathPlannerNode::popBacktrackTarget(Eigen::Vector3d *target) {
  // --- Backtracking selection ---
  // Pop the most recent valid backtrack target.
  while (!backtrack_stack_.empty()) {
    const Eigen::Vector3d candidate = backtrack_stack_.back();
    backtrack_stack_.pop_back();
    if ((candidate - current_position_).norm() >= backtrack_min_distance_) {
      *target = candidate;
      return true;
    }
  }
  return false;
}

bool PathPlannerNode::computeFrontierClusters(
    std::vector<FrontierCluster> *clusters) {
  // --- Frontier detection ---
  // Flood-fill free space and group frontier cells into clusters.
  clusters->clear();

  std::shared_ptr<octomap::OcTree> map;
  octomap::point3d min;
  octomap::point3d max;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = map_;
    min = map_min_;
    max = map_max_;
  }
  if (!map || !have_bounds_) {
    return false;
  }

  octomap::OcTreeKey start_key;
  if (!map->coordToKeyChecked(
          octomap::point3d(current_position_.x(), current_position_.y(),
                           current_position_.z()),
          start_key)) {
    return false;
  }

  if (!isFreeKeyInMap(map.get(), start_key, min, max, clearance_radius_)) {
    if (!findNearestFreeKey(start_key, &start_key)) {
      return false;
    }
  }

  const double res = map->getResolution();
  const int dirs[6][3] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                          {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};

  std::unordered_set<octomap::OcTreeKey, KeyHash, KeyEq> visited;
  std::unordered_set<octomap::OcTreeKey, KeyHash, KeyEq> frontier_cells;

  std::queue<octomap::OcTreeKey> queue;
  queue.push(start_key);
  visited.insert(start_key);

  const octomap::point3d start_coord = map->keyToCoord(start_key);

  while (!queue.empty() && static_cast<int>(visited.size()) < max_frontier_nodes_) {
    const octomap::OcTreeKey key = queue.front();
    queue.pop();

    const octomap::point3d coord = map->keyToCoord(key);
    if ((coord - start_coord).norm() > frontier_max_distance_) {
      continue;
    }

    if (isFrontierKeyInMap(map.get(), key, min, max)) {
      frontier_cells.insert(key);
    }

    for (const auto &d : dirs) {
      octomap::point3d ncoord(coord.x() + d[0] * res, coord.y() + d[1] * res,
                              coord.z() + d[2] * res);
      if (!coordInBounds(ncoord, min, max)) {
        continue;
      }
      octomap::OcTreeKey nkey;
      if (!map->coordToKeyChecked(ncoord, nkey)) {
        continue;
      }
      if (visited.find(nkey) != visited.end()) {
        continue;
      }
      if (!isFreeKeyInMap(map.get(), nkey, min, max, clearance_radius_)) {
        continue;
      }
      visited.insert(nkey);
      queue.push(nkey);
    }
  }

  if (frontier_cells.empty()) {
    return false;
  }

  std::unordered_set<octomap::OcTreeKey, KeyHash, KeyEq> frontier_visited;
  for (const auto &cell : frontier_cells) {
    if (frontier_visited.find(cell) != frontier_visited.end()) {
      continue;
    }

    FrontierCluster cluster;
    std::queue<octomap::OcTreeKey> cqueue;
    cqueue.push(cell);
    frontier_visited.insert(cell);

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();

    while (!cqueue.empty()) {
      const octomap::OcTreeKey ck = cqueue.front();
      cqueue.pop();
      cluster.cells.push_back(ck);
      const octomap::point3d ccoord = map->keyToCoord(ck);
      centroid += Eigen::Vector3d(ccoord.x(), ccoord.y(), ccoord.z());

      const octomap::point3d base = map->keyToCoord(ck);
      for (const auto &d : dirs) {
        octomap::point3d ncoord(base.x() + d[0] * res, base.y() + d[1] * res,
                                base.z() + d[2] * res);
        octomap::OcTreeKey nkey;
        if (!map->coordToKeyChecked(ncoord, nkey)) {
          continue;
        }
        if (frontier_cells.find(nkey) == frontier_cells.end()) {
          continue;
        }
        if (frontier_visited.find(nkey) != frontier_visited.end()) {
          continue;
        }
        frontier_visited.insert(nkey);
        cqueue.push(nkey);
      }
    }

    if (static_cast<int>(cluster.cells.size()) < frontier_min_cluster_size_) {
      continue;
    }
    centroid /= static_cast<double>(cluster.cells.size());
    cluster.centroid = centroid;
    clusters->push_back(cluster);
  }

  return !clusters->empty();
}

bool PathPlannerNode::selectFrontierGoal(
    const std::vector<FrontierCluster> &clusters,
    octomap::OcTreeKey *goal_key) {
  // --- Goal selection ---
  // Score clusters and pick the best goal candidate.
  if (clusters.empty()) {
    return false;
  }

  std::shared_ptr<octomap::OcTree> map;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = map_;
  }
  if (!map) {
    return false;
  }

  double best_score = -std::numeric_limits<double>::infinity();
  double best_score_any = -std::numeric_limits<double>::infinity();
  octomap::OcTreeKey best_key;
  octomap::OcTreeKey best_key_any;
  bool found = false;
  bool found_any = false;

  for (const auto &cluster : clusters) {
    if (cluster.cells.empty()) {
      continue;
    }
    const double dist = (cluster.centroid - current_position_).norm();
    if (dist < min_goal_distance_) {
      continue;
    }
    const double size = static_cast<double>(cluster.cells.size());
    const double forward_score = forward_bias_weight_ * forwardDot(cluster.centroid);
    const double lateral_penalty = lateral_bias_weight_ * lateralDistance(cluster.centroid);
    const double score = frontier_score_size_weight_ * size -
                         frontier_score_distance_weight_ * dist +
                         forward_score - lateral_penalty;

    double best_cell_dist = std::numeric_limits<double>::infinity();
    octomap::OcTreeKey candidate_key;
    bool candidate_found = false;
    for (const auto &cell : cluster.cells) {
      const octomap::point3d coord = map->keyToCoord(cell);
      const double d =
          (Eigen::Vector3d(coord.x(), coord.y(), coord.z()) - current_position_)
              .norm();
      if (d < min_goal_distance_) {
        continue;
      }
      if (d < best_cell_dist) {
        best_cell_dist = d;
        candidate_key = cell;
        candidate_found = true;
      }
    }
    if (!candidate_found) {
      continue;
    }

    if (score > best_score_any) {
      best_score_any = score;
      best_key_any = candidate_key;
      found_any = true;
    }

    if (min_forward_cos_ > -1.0 && forwardDot(cluster.centroid) < min_forward_cos_) {
      continue;
    }

    if (score > best_score) {
      best_score = score;
      best_key = candidate_key;
      found = true;
    }
  }

  if (!found && found_any) {
    best_key = best_key_any;
    found = true;
  }
  if (!found) {
    return false;
  }
  if (min_goal_distance_ > 0.0) {
    const octomap::point3d coord = map->keyToCoord(best_key);
    const double d =
        (Eigen::Vector3d(coord.x(), coord.y(), coord.z()) - current_position_)
            .norm();
    if (d < min_goal_distance_) {
      return false;
    }
  }
  *goal_key = best_key;
  return true;
}

double PathPlannerNode::forwardDot(const Eigen::Vector3d &target) const {
  const Eigen::Vector3d dir = target - current_position_;
  const double norm = dir.head<2>().norm();
  if (norm < 1e-6) {
    return 0.0;
  }
  const Eigen::Vector3d dir_xy(dir.x() / norm, dir.y() / norm, 0.0);
  return forward_dir_.dot(dir_xy);
}

double PathPlannerNode::lateralDistance(const Eigen::Vector3d &target) const {
  const Eigen::Vector3d dir = target - current_position_;
  const double norm = dir.head<2>().norm();
  if (norm < 1e-6) {
    return 0.0;
  }
  const Eigen::Vector3d dir_xy(dir.x() / norm, dir.y() / norm, 0.0);
  const double forward = std::max(-1.0, std::min(1.0, forward_dir_.dot(dir_xy)));
  const double sin_angle = std::sqrt(std::max(0.0, 1.0 - forward * forward));
  return norm * sin_angle;
}

bool PathPlannerNode::findNearestFreeKey(const octomap::OcTreeKey &seed_key,
                                         octomap::OcTreeKey *free_key) const {
  // --- Nearest-free search ---
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

  const double res = map->getResolution();
  const int dirs[6][3] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                          {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};

  std::queue<octomap::OcTreeKey> queue;
  std::unordered_set<octomap::OcTreeKey, KeyHash, KeyEq> visited;
  queue.push(seed_key);
  visited.insert(seed_key);

  while (!queue.empty() &&
         static_cast<int>(visited.size()) < max_frontier_nodes_) {
    const octomap::OcTreeKey key = queue.front();
    queue.pop();
    if (isFreeKeyInMap(map.get(), key, min, max, clearance_radius_)) {
      *free_key = key;
      return true;
    }
    const octomap::point3d coord = map->keyToCoord(key);
    for (const auto &d : dirs) {
      octomap::point3d ncoord(coord.x() + d[0] * res, coord.y() + d[1] * res,
                              coord.z() + d[2] * res);
      if (!coordInBounds(ncoord, min, max)) {
        continue;
      }
      octomap::OcTreeKey nkey;
      if (!map->coordToKeyChecked(ncoord, nkey)) {
        continue;
      }
      if (visited.find(nkey) != visited.end()) {
        continue;
      }
      visited.insert(nkey);
      queue.push(nkey);
    }
  }

  return false;
}
