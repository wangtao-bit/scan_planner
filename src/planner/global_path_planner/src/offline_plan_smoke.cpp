#include "global_path_planner/support_astar.hpp"
#include "global_path_planner/support_map.hpp"

#include <iostream>
#include <string>

int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " map.pcd\n";
    return 1;
  }
  global_path_planner::SupportMapParams mp;
  mp.resolution = 0.2;
  mp.min_clearance = 1.0;
  mp.robot_radius = 0.25;
  mp.robot_height = 0.4;
  mp.max_step_height = 0.40;
  mp.xy_snap_radius = 2.0;

  global_path_planner::SupportMap map;
  std::string err;
  if (!map.buildFromPcd(argv[1], mp, &err)) {
    std::cerr << "build failed: " << err << "\n";
    return 1;
  }
  std::cout << "map size " << map.size().transpose() << " res=" << map.resolution() << "\n";

  global_path_planner::SupportAstar planner;
  planner.setMap(&map);
  // Mode 3 demo endpoints on map.pcd (ground z).
  const auto result = planner.plan(
    Eigen::Vector3d(-5.5, 5.5, 0.10), Eigen::Vector3d(-5.5, -4.5, 1.55));
  std::cout << global_path_planner::planStatusToString(result.status) << ": " << result.message
            << "\n";
  if (result.status != global_path_planner::PlanStatus::Success) return 2;
  std::cout << "path points=" << result.path.size() << "\n";
  if (!result.path.empty()) {
    std::cout << "start z=" << result.path.front().z() << " end z=" << result.path.back().z()
              << "\n";
  }
  return 0;
}
