#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace global_path_planner
{

struct SupportMapParams
{
  double resolution{0.2};
  double min_clearance{1.0};
  double robot_radius{0.25};
  double robot_height{0.4};
  double max_step_height{0.40};
  double xy_snap_radius{1.0};
  /** Only accept goal supports within this |dz| of the goal height reference. */
  double goal_z_tolerance{0.6};
  /** Extra A* cost per incompatible 8-neighbor (stair/cliff edge preference). */
  double edge_cost_weight{0.4};
  /**
   * A neighbor whose closest support differs by more than this (m) is a step
   * or a cliff. Flat voxel noise is one resolution cell (~0.1 m), so keep this
   * above that and below a real riser.
   */
  double stair_rise{0.15};
  /** Soft keep-out (m) around a step/cliff edge. Cost falls to 0 at this distance. */
  double stair_margin{0.30};
  double pad_xy{1.0};
  double pad_z{0.5};
};

struct StairMarginStats
{
  int support_layers{0};
  /** Cells with a neighbor step: stair_rise < |dz| <= max_step_height. */
  int step_edge_cells{0};
  /** Cells with a neighbor drop/rise steeper than max_step_height. */
  int cliff_edge_cells{0};
  /** Supports whose proximity is > 0 (inside stair_margin of an edge). */
  int margin_cells{0};
  float edge_z_min{0.f};
  float edge_z_max{0.f};
};

/** Static occupancy + per-(x,y) support surfaces extracted from a PCD. */
class SupportMap
{
public:
  bool buildFromPcd(const std::string &pcd_file, const SupportMapParams &params, std::string *error);
  bool buildFromOccupied(
    const Eigen::Vector3d &origin, double resolution, const Eigen::Vector3i &size,
    const std::vector<uint8_t> &occupied, const SupportMapParams &params, std::string *error);

  bool isInMap(int ix, int iy) const;
  bool isOccupied(int ix, int iy, int iz) const;
  bool hasSupport(int ix, int iy) const;
  /** World-frame support height for layer k at (ix,iy). */
  float supportZ(int ix, int iy, int layer) const;
  int supportCount(int ix, int iy) const;

  /** Nearest traversable cell to (x,y,z_hint); layer chosen by closest support z. */
  bool snapToSupport(
    const Eigen::Vector3d &query, Eigen::Vector3i *cell_xy_layer, Eigen::Vector3d *world_pt,
    std::string *error = nullptr) const;

  /** Circular inflation check around body center at support + robot_height. */
  bool isBodyCollisionFree(int ix, int iy, int layer) const;

  /**
   * Count of 8-neighbors that lack a support within max_step_height of this
   * cell's support z (missing / cliff / other floor). Used as edge cost.
   */
  int incompatibleNeighborCount(int ix, int iy, int layer) const;

  /**
   * 1 on a step/cliff edge, falling linearly to 0 at stair_margin.
   * Only supports on the same floor (within max_step_height) are painted.
   */
  float stairProximity(int ix, int iy, int layer) const;
  float stairProximityAt(const Eigen::Vector3d &pt) const;
  const StairMarginStats &stairMarginStats() const { return stair_stats_; }

  const SupportMapParams &params() const { return params_; }
  const Eigen::Vector3d &origin() const { return origin_; }
  const Eigen::Vector3i &size() const { return size_; }
  double resolution() const { return resolution_; }

  Eigen::Vector3d indexToWorld(int ix, int iy, int iz) const;
  bool worldToIndex(const Eigen::Vector3d &pt, Eigen::Vector3i *idx) const;

  /** Flattened support cloud for visualization (x,y,z). */
  std::vector<Eigen::Vector3d> supportCloud() const;

private:
  void extractSupports();
  void buildStairMargin();
  int flatXY(int ix, int iy) const { return iy * size_.x() + ix; }
  int flatXYZ(int ix, int iy, int iz) const
  {
    return iz * size_.x() * size_.y() + iy * size_.x() + ix;
  }
  bool columnClearAbove(int ix, int iy, int iz_top_occupied) const;

  SupportMapParams params_;
  double resolution_{0.2};
  Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};
  Eigen::Vector3i size_{Eigen::Vector3i::Zero()};
  std::vector<uint8_t> occupied_;
  /** For each (ix,iy): list of support world-z values (ascending). */
  std::vector<std::vector<float>> supports_;
  /** Parallel to supports_: [0, 1] stair keep-out, 1 = on a step/cliff edge. */
  std::vector<std::vector<float>> stair_proximity_;
  StairMarginStats stair_stats_;
};

}  // namespace global_path_planner
