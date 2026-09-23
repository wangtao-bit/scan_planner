#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "global_path_planner/support_map.hpp"

namespace global_path_planner
{

enum class PlanStatus
{
  Success = 0,
  StartSnapFailed,
  GoalSnapFailed,
  SearchFailed
};

struct PlanResult
{
  PlanStatus status{PlanStatus::SearchFailed};
  std::string message;
  std::vector<Eigen::Vector3d> path;  // ground / support z
};

class SupportAstar
{
public:
  void setMap(const SupportMap *map) { map_ = map; }

  PlanResult plan(const Eigen::Vector3d &start, const Eigen::Vector3d &goal) const;

  /** Downsample polyline so consecutive points are at least min_distance apart; keep end. */
  static std::vector<Eigen::Vector3d> downsample(
    const std::vector<Eigen::Vector3d> &path, double min_distance);

private:
  const SupportMap *map_{nullptr};
};

inline const char *planStatusToString(PlanStatus status)
{
  switch (status) {
    case PlanStatus::Success:
      return "Success";
    case PlanStatus::StartSnapFailed:
      return "StartSnapFailed";
    case PlanStatus::GoalSnapFailed:
      return "GoalSnapFailed";
    case PlanStatus::SearchFailed:
      return "SearchFailed";
  }
  return "Unknown";
}

}  // namespace global_path_planner
