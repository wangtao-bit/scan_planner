#include "global_path_planner/support_astar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>

namespace global_path_planner
{
namespace
{

struct NodeKey
{
  int x{0};
  int y{0};
  int layer{0};
  bool operator==(const NodeKey &o) const
  {
    return x == o.x && y == o.y && layer == o.layer;
  }
};

struct NodeKeyHash
{
  size_t operator()(const NodeKey &k) const
  {
    return (static_cast<size_t>(k.x) * 73856093u) ^
           (static_cast<size_t>(k.y) * 19349663u) ^ (static_cast<size_t>(k.layer) * 83492791u);
  }
};

struct OpenItem
{
  double f{0.0};
  double g{0.0};
  NodeKey key;
};

struct OpenCompare
{
  bool operator()(const OpenItem &a, const OpenItem &b) const { return a.f > b.f; }
};

}  // namespace

std::vector<Eigen::Vector3d> SupportAstar::downsample(
  const std::vector<Eigen::Vector3d> &path, double min_distance)
{
  if (path.size() <= 2) return path;
  std::vector<Eigen::Vector3d> out;
  out.push_back(path.front());
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    if ((path[i] - out.back()).norm() >= min_distance) out.push_back(path[i]);
  }
  if ((path.back() - out.back()).norm() > 1e-6) out.push_back(path.back());
  if (out.size() < 2) out.push_back(path.back());
  return out;
}

PlanResult SupportAstar::plan(const Eigen::Vector3d &start, const Eigen::Vector3d &goal) const
{
  PlanResult result;
  if (!map_) {
    result.message = "map not set";
    return result;
  }

  Eigen::Vector3i start_cell;
  Eigen::Vector3d start_pt;
  std::string err;
  if (!map_->snapToSupport(start, &start_cell, &start_pt, &err)) {
    result.status = PlanStatus::StartSnapFailed;
    result.message = "start: " + err;
    return result;
  }

  // Collect candidates near goal XY whose support z is close to the requested
  // height (Publish Point z, or start height for flat 2D Goal Pose z≈0).
  const double res = map_->resolution();
  const int snap_cells =
    std::max(1, static_cast<int>(std::ceil(map_->params().xy_snap_radius / res)));
  Eigen::Vector3i gidx;
  map_->worldToIndex(goal, &gidx);

  const bool flat_goal = std::fabs(goal.z()) < 0.05;
  const double z_ref = flat_goal ? start_pt.z() : goal.z();
  const double z_tol = std::max(0.05, map_->params().goal_z_tolerance);

  struct GoalCand
  {
    NodeKey key;
    Eigen::Vector3d pt;
    double pref{0.0};
  };
  std::vector<GoalCand> goals;
  for (int dy = -snap_cells; dy <= snap_cells; ++dy) {
    for (int dx = -snap_cells; dx <= snap_cells; ++dx) {
      const int ix = gidx.x() + dx;
      const int iy = gidx.y() + dy;
      if (!map_->hasSupport(ix, iy)) continue;
      for (int k = 0; k < map_->supportCount(ix, iy); ++k) {
        if (!map_->isBodyCollisionFree(ix, iy, k)) continue;
        const double z = map_->supportZ(ix, iy, k);
        if (std::fabs(z - z_ref) > z_tol) continue;
        const Eigen::Vector3d pt(
          map_->origin().x() + (ix + 0.5) * res, map_->origin().y() + (iy + 0.5) * res, z);
        const double xy = (pt.head<2>() - goal.head<2>()).norm();
        if (xy > map_->params().xy_snap_radius) continue;
        const double pref = xy + 0.25 * std::fabs(z - z_ref);
        goals.push_back(GoalCand{NodeKey{ix, iy, k}, pt, pref});
      }
    }
  }
  if (goals.empty()) {
    result.status = PlanStatus::GoalSnapFailed;
    result.message =
      "goal: no support near XY within z tolerance of " + std::to_string(z_ref);
    return result;
  }
  std::sort(goals.begin(), goals.end(), [](const GoalCand &a, const GoalCand &b) {
    return a.pref < b.pref;
  });

  std::unordered_map<NodeKey, bool, NodeKeyHash> is_goal;
  for (const auto &g : goals) is_goal[g.key] = true;

  const NodeKey start_key{start_cell.x(), start_cell.y(), start_cell.z()};
  if (is_goal[start_key]) {
    result.status = PlanStatus::Success;
    result.message = "start equals goal";
    // Pick preferred goal point at start cell if present.
    result.path = {start_pt, start_pt};
    for (const auto &g : goals) {
      if (g.key == start_key) {
        result.path.back() = g.pt;
        break;
      }
    }
    return result;
  }

  const double max_step = map_->params().max_step_height;
  const int dx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  const int dy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  const double cost8[8] = {1, 1.41421356, 1, 1.41421356, 1, 1.41421356, 1, 1.41421356};

  // Heuristic toward the preferred (first) goal candidate.
  const Eigen::Vector3d prefer_pt = goals.front().pt;
  auto heuristic = [&](const NodeKey &a) {
    const double ax = map_->origin().x() + (a.x + 0.5) * res;
    const double ay = map_->origin().y() + (a.y + 0.5) * res;
    const double az = map_->supportZ(a.x, a.y, a.layer);
    return std::hypot(ax - prefer_pt.x(), ay - prefer_pt.y()) +
           0.25 * std::fabs(az - prefer_pt.z());
  };

  std::priority_queue<OpenItem, std::vector<OpenItem>, OpenCompare> open;
  std::unordered_map<NodeKey, double, NodeKeyHash> g_score;
  std::unordered_map<NodeKey, NodeKey, NodeKeyHash> parent;
  std::unordered_map<NodeKey, bool, NodeKeyHash> closed;

  g_score[start_key] = 0.0;
  open.push(OpenItem{heuristic(start_key), 0.0, start_key});

  NodeKey found_goal{-1, -1, -1};
  bool found = false;
  while (!open.empty()) {
    const OpenItem cur = open.top();
    open.pop();
    if (closed[cur.key]) continue;
    if (cur.g > g_score[cur.key] + 1e-9) continue;
    closed[cur.key] = true;

    if (is_goal[cur.key]) {
      found = true;
      found_goal = cur.key;
      break;
    }

    const float z_cur = map_->supportZ(cur.key.x, cur.key.y, cur.key.layer);
    for (int n = 0; n < 8; ++n) {
      const int nx = cur.key.x + dx8[n];
      const int ny = cur.key.y + dy8[n];
      if (!map_->hasSupport(nx, ny)) continue;
      for (int k = 0; k < map_->supportCount(nx, ny); ++k) {
        const float z_next = map_->supportZ(nx, ny, k);
        if (std::fabs(z_next - z_cur) > max_step) continue;
        if (!map_->isBodyCollisionFree(nx, ny, k)) continue;
        const NodeKey next{nx, ny, k};
        if (closed[next]) continue;
        const double edge_pen =
          map_->params().edge_cost_weight * res *
          static_cast<double>(map_->incompatibleNeighborCount(nx, ny, k));
        const double tentative =
          cur.g + cost8[n] * res + 0.1 * std::fabs(z_next - z_cur) + edge_pen;
        auto it = g_score.find(next);
        if (it != g_score.end() && tentative >= it->second) continue;
        g_score[next] = tentative;
        parent[next] = cur.key;
        open.push(OpenItem{tentative + heuristic(next), tentative, next});
      }
    }
  }

  if (!found) {
    result.status = PlanStatus::SearchFailed;
    result.message = "A* failed to reach any nearby goal support";
    return result;
  }

  std::vector<NodeKey> keys;
  for (NodeKey k = found_goal;; k = parent[k]) {
    keys.push_back(k);
    if (k == start_key) break;
    if (parent.find(k) == parent.end()) {
      result.status = PlanStatus::SearchFailed;
      result.message = "broken parent chain";
      return result;
    }
  }
  std::reverse(keys.begin(), keys.end());

  Eigen::Vector3d goal_pt = prefer_pt;
  for (const auto &g : goals) {
    if (g.key == found_goal) {
      goal_pt = g.pt;
      break;
    }
  }

  result.path.clear();
  result.path.reserve(keys.size());
  for (const auto &k : keys) {
    result.path.emplace_back(
      map_->origin().x() + (k.x + 0.5) * res, map_->origin().y() + (k.y + 0.5) * res,
      map_->supportZ(k.x, k.y, k.layer));
  }
  if (!result.path.empty()) {
    result.path.front() = start_pt;
    result.path.back() = goal_pt;
  }
  result.path = downsample(result.path, 0.5);
  result.status = PlanStatus::Success;
  result.message = "ok, points=" + std::to_string(result.path.size());
  return result;
}

}  // namespace global_path_planner
