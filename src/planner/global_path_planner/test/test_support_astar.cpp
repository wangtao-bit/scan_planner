#include "global_path_planner/support_astar.hpp"
#include "global_path_planner/support_map.hpp"

#include <cmath>
#include <gtest/gtest.h>

#include <vector>

using global_path_planner::PlanStatus;
using global_path_planner::SupportAstar;
using global_path_planner::SupportMap;
using global_path_planner::SupportMapParams;

TEST(SupportMapTest, FlatFloorPlan)
{
  // 4m x 4m floor @ 0.2m: floor at iz=0, free above; wall blocks mid corridor.
  const int nx = 20, ny = 20, nz = 10;
  const double res = 0.2;
  Eigen::Vector3d origin(0.0, 0.0, 0.0);
  std::vector<uint8_t> occupied(static_cast<size_t>(nx) * ny * nz, 0);
  for (int iy = 0; iy < ny; ++iy) {
    for (int ix = 0; ix < nx; ++ix) {
      occupied[static_cast<size_t>(0) * nx * ny + iy * nx + ix] = 1;
    }
  }
  // Solid wall across most of the map at x≈2m, leaving a gap at low y.
  for (int iy = 4; iy < ny; ++iy) {
    for (int iz = 1; iz < 6; ++iz) {
      occupied[static_cast<size_t>(iz) * nx * ny + iy * nx + 10] = 1;
    }
  }

  SupportMapParams params;
  params.resolution = res;
  params.min_clearance = 0.8;
  params.robot_radius = 0.15;
  params.robot_height = 0.4;
  params.max_step_height = 0.3;
  params.xy_snap_radius = 1.0;

  SupportMap map;
  std::string err;
  ASSERT_TRUE(map.buildFromOccupied(origin, res, Eigen::Vector3i(nx, ny, nz), occupied, params, &err))
    << err;
  EXPECT_TRUE(map.hasSupport(2, 2));

  SupportAstar planner;
  planner.setMap(&map);
  const auto result = planner.plan(Eigen::Vector3d(0.5, 2.0, 0.2), Eigen::Vector3d(3.5, 2.0, 0.2));
  EXPECT_EQ(result.status, PlanStatus::Success) << result.message;
  ASSERT_GE(result.path.size(), 2u);
  // Must go through the gap at low y to pass the wall.
  bool used_gap = false;
  for (const auto &p : result.path) {
    if (p.y() < 0.9) used_gap = true;
  }
  EXPECT_TRUE(used_gap);
}

TEST(SupportAstarTest, DownsampleKeepsEnds)
{
  std::vector<Eigen::Vector3d> path = {
    {0, 0, 0}, {0.1, 0, 0}, {0.6, 0, 0}, {1.2, 0, 0}};
  const auto out = SupportAstar::downsample(path, 0.5);
  ASSERT_GE(out.size(), 2u);
  EXPECT_NEAR(out.front().x(), 0.0, 1e-9);
  EXPECT_NEAR(out.back().x(), 1.2, 1e-9);
}
