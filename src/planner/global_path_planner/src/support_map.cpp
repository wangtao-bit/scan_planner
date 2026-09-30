#include "global_path_planner/support_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace global_path_planner
{

bool SupportMap::buildFromPcd(
  const std::string &pcd_file, const SupportMapParams &params, std::string *error)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(pcd_file, cloud) != 0 || cloud.empty()) {
    if (error) *error = "failed to load PCD: " + pcd_file;
    return false;
  }

  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(cloud, min_pt, max_pt);
  const double res = params.resolution;
  if (!(res > 1e-6)) {
    if (error) *error = "resolution must be positive";
    return false;
  }
  // 原点
  Eigen::Vector3d origin(
    min_pt.x - params.pad_xy, min_pt.y - params.pad_xy, min_pt.z - params.pad_z);
  const int nx = static_cast<int>(std::ceil((max_pt.x + params.pad_xy - origin.x()) / res));
  const int ny = static_cast<int>(std::ceil((max_pt.y + params.pad_xy - origin.y()) / res));
  const int nz = static_cast<int>(std::ceil((max_pt.z + params.pad_z - origin.z()) / res));
  if (nx <= 0 || ny <= 0 || nz <= 0) {
    if (error) *error = "invalid map size from PCD bounds";
    return false;
  }

  std::vector<uint8_t> occupied(static_cast<size_t>(nx) * ny * nz, 0);
  const double inv_res = 1.0 / res;
  for (const auto &p : cloud) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    const int ix = static_cast<int>(std::floor((p.x - origin.x()) * inv_res));
    const int iy = static_cast<int>(std::floor((p.y - origin.y()) * inv_res));
    const int iz = static_cast<int>(std::floor((p.z - origin.z()) * inv_res));
    if (ix < 0 || iy < 0 || iz < 0 || ix >= nx || iy >= ny || iz >= nz) continue;
    // 一维索引
    occupied[static_cast<size_t>(iz) * nx * ny + static_cast<size_t>(iy) * nx + ix] = 1;
  }
  // 
  return buildFromOccupied(origin, res, Eigen::Vector3i(nx, ny, nz), occupied, params, error);
}

bool SupportMap::buildFromOccupied(
  const Eigen::Vector3d &origin, double resolution, const Eigen::Vector3i &size,
  const std::vector<uint8_t> &occupied, const SupportMapParams &params, std::string *error)
{
  if (resolution <= 1e-6 || size.x() <= 0 || size.y() <= 0 || size.z() <= 0) {
    if (error) *error = "invalid occupied grid dimensions";
    return false;
  }
  const size_t expected =
    static_cast<size_t>(size.x()) * static_cast<size_t>(size.y()) * static_cast<size_t>(size.z());
  if (occupied.size() != expected) {
    if (error) *error = "occupied buffer size mismatch";
    return false;
  }

  params_ = params;
  resolution_ = resolution;
  origin_ = origin;
  size_ = size;
  occupied_ = occupied;
  supports_.assign(static_cast<size_t>(size_.x()) * size_.y(), {});
  extractSupports();
  buildStairMargin();
  return true;
}
// 提取支撑面
void SupportMap::extractSupports()
{
  // 撑面上方必须留空的净空高度（米）
  const int clearance_cells =
    std::max(1, static_cast<int>(std::ceil(params_.min_clearance / resolution_)));

  for (int iy = 0; iy < size_.y(); ++iy) {
    for (int ix = 0; ix < size_.x(); ++ix) {
      auto &layers = supports_[static_cast<size_t>(flatXY(ix, iy))];
      layers.clear();
      for (int iz = 0; iz < size_.z(); ++iz) {
        if (!isOccupied(ix, iy, iz)) continue;
        // Top of a contiguous occupied stack: next z free (or map top).
        const bool top_of_stack =
          (iz + 1 >= size_.z()) || !isOccupied(ix, iy, iz + 1);
        if (!top_of_stack) continue;
        // 支撑面上方必须留空的净空高度（米）
        if (!columnClearAbove(ix, iy, iz)) continue;
        // Require enough free cells above the support top.
        bool clear = true;
        for (int dz = 1; dz <= clearance_cells; ++dz) {
          const int jz = iz + dz;
          if (jz >= size_.z()) break;
          if (isOccupied(ix, iy, jz)) {
            clear = false;
            break;
          }
        }
        if (!clear) continue;
        const float z = static_cast<float>(origin_.z() + (iz + 1) * resolution_);
        if (layers.empty() || std::fabs(layers.back() - z) > 0.5f * resolution_) {
          layers.push_back(z);
        }
      }
    }
  }
}
// 检查撑面上方是否留空
bool SupportMap::columnClearAbove(int ix, int iy, int iz_top_occupied) const
{
  const int clearance_cells =
    std::max(1, static_cast<int>(std::ceil(params_.min_clearance / resolution_)));
  for (int dz = 1; dz <= clearance_cells; ++dz) {
    const int jz = iz_top_occupied + dz;
    if (jz >= size_.z()) return true;
    if (isOccupied(ix, iy, jz)) return false;
  }
  return true;
}

bool SupportMap::isInMap(int ix, int iy) const
{
  return ix >= 0 && iy >= 0 && ix < size_.x() && iy < size_.y();
}

bool SupportMap::isOccupied(int ix, int iy, int iz) const
{
  if (!isInMap(ix, iy) || iz < 0 || iz >= size_.z()) return true;
  return occupied_[static_cast<size_t>(flatXYZ(ix, iy, iz))] != 0;
}

bool SupportMap::hasSupport(int ix, int iy) const
{
  if (!isInMap(ix, iy)) return false;
  return !supports_[static_cast<size_t>(flatXY(ix, iy))].empty();
}

float SupportMap::supportZ(int ix, int iy, int layer) const
{
  return supports_[static_cast<size_t>(flatXY(ix, iy))][static_cast<size_t>(layer)];
}

int SupportMap::supportCount(int ix, int iy) const
{
  if (!isInMap(ix, iy)) return 0;
  return static_cast<int>(supports_[static_cast<size_t>(flatXY(ix, iy))].size());
}

Eigen::Vector3d SupportMap::indexToWorld(int ix, int iy, int iz) const
{
  return Eigen::Vector3d(
    origin_.x() + (ix + 0.5) * resolution_, origin_.y() + (iy + 0.5) * resolution_,
    origin_.z() + (iz + 0.5) * resolution_);
}

bool SupportMap::worldToIndex(const Eigen::Vector3d &pt, Eigen::Vector3i *idx) const
{
  if (!idx) return false;
  idx->x() = static_cast<int>(std::floor((pt.x() - origin_.x()) / resolution_));
  idx->y() = static_cast<int>(std::floor((pt.y() - origin_.y()) / resolution_));
  idx->z() = static_cast<int>(std::floor((pt.z() - origin_.z()) / resolution_));
  return isInMap(idx->x(), idx->y()) && idx->z() >= 0 && idx->z() < size_.z();
}
// 碰撞检测
bool SupportMap::isBodyCollisionFree(int ix, int iy, int layer) const
{
  if (!hasSupport(ix, iy) || layer < 0 || layer >= supportCount(ix, iy)) return false;
  const double support_z = supportZ(ix, iy, layer);
  // Check a horizontal disk at roughly body mid-height for obstacle intrusion.
  const double check_z = support_z + params_.robot_height;
  const int jz = static_cast<int>(std::floor((check_z - origin_.z()) / resolution_));
  if (jz < 0 || jz >= size_.z()) return false;
  const int r_cells = std::max(0, static_cast<int>(std::ceil(params_.robot_radius / resolution_)));

  for (int dy = -r_cells; dy <= r_cells; ++dy) {
    for (int dx = -r_cells; dx <= r_cells; ++dx) {
      if (dx * dx + dy * dy > r_cells * r_cells) continue;
      const int jx = ix + dx;
      const int jy = iy + dy;
      if (!isInMap(jx, jy)) return false;
      if (isOccupied(jx, jy, jz)) return false;
    }
  }
  return true;
}

void SupportMap::buildStairMargin()
{
  // 初始化
  stair_proximity_.assign(supports_.size(), {});
  stair_stats_ = StairMarginStats{};
  bool have_edge_z = false;
  // 高度差超过 rise 的台阶边缘
  const double rise = params_.stair_rise;
  const double margin = params_.stair_margin;
  // 最大高度差
  const double max_step = params_.max_step_height;
  // margin对应的栅格数
  const int rad =
    margin > 1e-6 ? std::max(0, static_cast<int>(std::ceil(margin / resolution_))) : 0;
  // 台阶边缘点
  struct Source
  {
    int ix{0};
    int iy{0};
    float z{0.f};
  };
  std::vector<Source> sources;
  sources.reserve(1024);
  static const int dx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  static const int dy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};

  for (int iy = 0; iy < size_.y(); ++iy) {
    for (int ix = 0; ix < size_.x(); ++ix) {
      const int nlayer = supportCount(ix, iy);
      stair_proximity_[static_cast<size_t>(flatXY(ix, iy))].assign(
        static_cast<size_t>(nlayer), 0.f);
      stair_stats_.support_layers += nlayer;
      for (int k = 0; k < nlayer; ++k) {
        const float z = supportZ(ix, iy, k);
        bool step_edge = false;
        bool cliff_edge = false;
        for (int n = 0; n < 8; ++n) {
          const int jx = ix + dx8[n];
          const int jy = iy + dy8[n];
          if (!hasSupport(jx, jy)) continue;
          float closest = std::numeric_limits<float>::infinity();
          for (int kk = 0; kk < supportCount(jx, jy); ++kk) {
            closest = std::min(closest, std::fabs(supportZ(jx, jy, kk) - z));
          }
          if (closest > static_cast<float>(max_step)) {
            cliff_edge = true;
          } else if (closest > static_cast<float>(rise)) {
            step_edge = true;
          }
        }
        if (!step_edge && !cliff_edge) continue;
        if (step_edge) ++stair_stats_.step_edge_cells;
        if (cliff_edge) ++stair_stats_.cliff_edge_cells;
        if (!have_edge_z) {
          stair_stats_.edge_z_min = stair_stats_.edge_z_max = z;
          have_edge_z = true;
        } else {
          stair_stats_.edge_z_min = std::min(stair_stats_.edge_z_min, z);
          stair_stats_.edge_z_max = std::max(stair_stats_.edge_z_max, z);
        }
        sources.push_back(Source{ix, iy, z});
      }
    }
  }

  if (rad == 0 || sources.empty()) return;

  for (const auto &src : sources) {
    for (int dy = -rad; dy <= rad; ++dy) {
      for (int dx = -rad; dx <= rad; ++dx) {
        const double dist = std::hypot(static_cast<double>(dx), static_cast<double>(dy)) * resolution_;
        if (dist > margin) continue;
        const int jx = src.ix + dx;
        const int jy = src.iy + dy;
        if (!hasSupport(jx, jy)) continue;
        const float prox = static_cast<float>(std::max(0.0, 1.0 - dist / margin));
        auto &layers = stair_proximity_[static_cast<size_t>(flatXY(jx, jy))];
        for (int k = 0; k < supportCount(jx, jy); ++k) {
          if (std::fabs(supportZ(jx, jy, k) - src.z) > max_step) continue;
          layers[static_cast<size_t>(k)] = std::max(layers[static_cast<size_t>(k)], prox);
        }
      }
    }
  }

  for (const auto &col : stair_proximity_) {
    for (const float prox : col) {
      if (prox > 1e-4f) ++stair_stats_.margin_cells;
    }
  }
}

float SupportMap::stairProximity(int ix, int iy, int layer) const
{
  if (!hasSupport(ix, iy) || layer < 0 || layer >= supportCount(ix, iy)) return 0.f;
  const auto &col = stair_proximity_[static_cast<size_t>(flatXY(ix, iy))];
  if (layer >= static_cast<int>(col.size())) return 0.f;
  return col[static_cast<size_t>(layer)];
}

float SupportMap::stairProximityAt(const Eigen::Vector3d &pt) const
{
  if (!(resolution_ > 1e-6)) return 0.f;
  const int ix = static_cast<int>(std::floor((pt.x() - origin_.x()) / resolution_));
  const int iy = static_cast<int>(std::floor((pt.y() - origin_.y()) / resolution_));
  if (!hasSupport(ix, iy)) return 0.f;
  int best_k = -1;
  float best_dz = std::numeric_limits<float>::infinity();
  for (int k = 0; k < supportCount(ix, iy); ++k) {
    const float dz = std::fabs(supportZ(ix, iy, k) - static_cast<float>(pt.z()));
    if (dz < best_dz) {
      best_dz = dz;
      best_k = k;
    }
  }
  if (best_k < 0 || best_dz > static_cast<float>(params_.max_step_height)) return 0.f;
  return stairProximity(ix, iy, best_k);
}

int SupportMap::incompatibleNeighborCount(int ix, int iy, int layer) const
{
  if (!hasSupport(ix, iy) || layer < 0 || layer >= supportCount(ix, iy)) return 8;
  const float z = supportZ(ix, iy, layer);
  const double max_step = params_.max_step_height;
  static const int dx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  static const int dy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  int bad = 0;
  for (int n = 0; n < 8; ++n) {
    const int jx = ix + dx8[n];
    const int jy = iy + dy8[n];
    // 检查邻居是否也有支撑面
    if (!hasSupport(jx, jy)) {
      ++bad;
      continue;
    }
    bool compatible = false;
    for (int k = 0; k < supportCount(jx, jy); ++k) {
      if (std::fabs(supportZ(jx, jy, k) - z) <= max_step) {
        compatible = true;
        break;
      }
    }
    if (!compatible) ++bad;
  }
  return bad;
}

bool SupportMap::snapToSupport(
  const Eigen::Vector3d &query, Eigen::Vector3i *cell_xy_layer, Eigen::Vector3d *world_pt,
  std::string *error) const
{
  if (!cell_xy_layer || !world_pt) {
    if (error) *error = "null output";
    return false;
  }
  const int snap_cells =
    std::max(1, static_cast<int>(std::ceil(params_.xy_snap_radius / resolution_)));
  Eigen::Vector3i qidx;
  worldToIndex(query, &qidx);

  double best_cost = std::numeric_limits<double>::infinity();
  Eigen::Vector3i best(-1, -1, -1);
  Eigen::Vector3d best_pt = query;

  for (int dy = -snap_cells; dy <= snap_cells; ++dy) {
    for (int dx = -snap_cells; dx <= snap_cells; ++dx) {
      const int ix = qidx.x() + dx;
      const int iy = qidx.y() + dy;
      if (!hasSupport(ix, iy)) continue;
      for (int k = 0; k < supportCount(ix, iy); ++k) {
        if (!isBodyCollisionFree(ix, iy, k)) continue;
        const double z = supportZ(ix, iy, k);
        const Eigen::Vector3d cand(
          origin_.x() + (ix + 0.5) * resolution_, origin_.y() + (iy + 0.5) * resolution_, z);
        const double xy = (cand.head<2>() - query.head<2>()).norm();
        const double dz = std::fabs(cand.z() - query.z());
        const double cost = xy + 0.5 * dz;
        if (cost < best_cost) {
          best_cost = cost;
          best = Eigen::Vector3i(ix, iy, k);
          best_pt = cand;
        }
      }
    }
  }

  if (best.x() < 0) {
    if (error) *error = "no nearby support surface";
    return false;
  }
  *cell_xy_layer = best;
  *world_pt = best_pt;
  return true;
}

std::vector<Eigen::Vector3d> SupportMap::supportCloud() const
{
  std::vector<Eigen::Vector3d> cloud;
  cloud.reserve(static_cast<size_t>(size_.x()) * size_.y());
  for (int iy = 0; iy < size_.y(); ++iy) {
    for (int ix = 0; ix < size_.x(); ++ix) {
      for (int k = 0; k < supportCount(ix, iy); ++k) {
        cloud.emplace_back(
          origin_.x() + (ix + 0.5) * resolution_, origin_.y() + (iy + 0.5) * resolution_,
          supportZ(ix, iy, k));
      }
    }
  }
  return cloud;
}

}  // namespace global_path_planner
