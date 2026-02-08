#pragma once

#include <Eigen/Dense>

#include <octomap/octomap.h>

#include <cstddef>
#include <vector>

struct FrontierCluster {
  std::vector<octomap::OcTreeKey> cells;
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
};

struct KeyHash {
  std::size_t operator()(const octomap::OcTreeKey &key) const noexcept;
};

struct KeyEq {
  bool operator()(const octomap::OcTreeKey &a,
                  const octomap::OcTreeKey &b) const noexcept;
};
