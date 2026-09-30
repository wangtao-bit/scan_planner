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
  bool have_start_snap{false};
  Eigen::Vector3d snapped_start{Eigen::Vector3d::Zero()};
  double start_stair_proximity{0.0};
  int goal_candidate_count{0};
  bool have_preferred_goal{false};
  Eigen::Vector3d preferred_goal{Eigen::Vector3d::Zero()};
  double preferred_goal_stair_proximity{0.0};
  double goal_z_reference{0.0};
  double path_max_stair_proximity{0.0};
  int path_points_inside_margin{0};
  double path_length{0.0};
  /** Up to a few A* cells with stair proximity >= 0.5, for the terminal log. */
  std::string stair_contact;
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
