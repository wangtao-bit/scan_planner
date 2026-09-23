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
  double pad_xy{1.0};
  double pad_z{0.5};
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
};

}  // namespace global_path_planner
