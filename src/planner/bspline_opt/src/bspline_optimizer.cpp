#include "bspline_opt/bspline_optimizer.h"
#include "bspline_opt/gradient_descent_optimizer.h"
#include <algorithm>
#include <chrono>
#include <cmath>
// using namespace std;

namespace scan_planner
{

  void BsplineOptimizer::setParam(rclcpp::Node *node)
  {
    const auto get_double = [node](const std::string &name, double default_value) {
      if (!node->has_parameter(name)) node->declare_parameter<double>(name, default_value);
      return node->get_parameter(name).as_double();
    };
    lambda1_ = get_double("optimization.lambda_smooth", -1.0);
    lambda2_ = get_double("optimization.lambda_collision", -1.0);
    lambda3_ = get_double("optimization.lambda_feasibility", -1.0);
    lambda4_ = get_double("optimization.lambda_fitness", -1.0);
    dist0_ = get_double("optimization.dist0", -1.0);
    max_vel_ = get_double("optimization.max_vel", -1.0);
    max_acc_ = get_double("optimization.max_acc", -1.0);
    corridor_width_ = get_double("optimization.corridor_width", 1.0);
    corridor_max_range_ = get_double("optimization.corridor_max_range", 1.5);
    corridor_margin_ = get_double("optimization.corridor_margin", 0.05);
    lambda_corridor_ = get_double("optimization.lambda_corridor", 1.0);
    lambda_clearance_ = get_double("optimization.lambda_clearance", 0.05);
    RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                "[corridor] params width=%.3f max_range=%.3f margin=%.3f lambda_cor=%.3f lambda_clr=%.3f dist0=%.3f",
                corridor_width_, corridor_max_range_, corridor_margin_, lambda_corridor_, lambda_clearance_, dist0_);
    if (!node->has_parameter("optimization.order")) node->declare_parameter<int>("optimization.order", 3);
    order_ = static_cast<int>(node->get_parameter("optimization.order").as_int());
  }

  void BsplineOptimizer::setEnvironment(const GridMap::Ptr &env)
  {
    this->grid_map_ = env;
  }

  double BsplineOptimizer::estimateSegmentYaw(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(diff(1), diff(0));
  }

  double BsplineOptimizer::estimateControlPointYaw(const Eigen::MatrixXd &q, int id) const
  {
    if (q.cols() <= 1)
      return 0.0;

    int prev_id = std::max(0, id - 1);
    int next_id = std::min(static_cast<int>(q.cols()) - 1, id + 1);
    return estimateSegmentYaw(q.col(prev_id), q.col(next_id));
  }

  void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.points = points;
  }

  void BsplineOptimizer::setBsplineInterval(const double &ts) { bspline_interval_ = ts; }

  /* This function is very similar to check_collision_and_rebound(). 
   * It was written separately, just because I did it once and it has been running stably since March 2020.
   * But I will merge then someday.*/
  std::vector<std::vector<Eigen::Vector3d>> BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/)
  {

    // 初始化控制点
    if (flag_first_init)
    {
      cps_.clearance = dist0_;
      cps_.resize(init_points.cols());
      cps_.points = init_points;
    }

    /*** Segment the initial trajectory according to obstacles ***/
    constexpr int ENOUGH_INTERVAL = 2;
    double step_size = grid_map_->getResolution() / ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 2;
    int in_id = -1, out_id = -1;
    vector<std::pair<int, int>> segment_ids;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    int i_end = (int)init_points.cols() - order_ - ((int)init_points.cols() - 2 * order_) / 3; // only check closed 2/3 points.
    for (int i = order_; i <= i_end; ++i)
    {
      for (double a = 1.0; a >= 0.0; a -= step_size)
      {
        // 线性插值
        Eigen::Vector3d sample_pt = a * init_points.col(i - 1) + (1 - a) * init_points.col(i);
        // 估计航向角
        double sample_yaw = estimateSegmentYaw(init_points.col(i - 1), init_points.col(i));
        // 碰撞检测
        occ = grid_map_->getInflateOccupancy(sample_pt, sample_yaw);
        // cout << setprecision(5);
        // cout << (a * init_points.col(i-1) + (1-a) * init_points.col(i)).transpose() << " occ1=" << occ << endl;
        // in_id: 起点
        // out_id: 终点
        // flag_got_start: 是否已经找到起点
        // flag_got_end: 是否已经找到终点
        // flag_got_end_maybe: 是否可能找到终点
        // same_occ_state_times: 连续碰撞状态的次数
        // last_occ: 上一个状态是否碰撞
        // occ: 当前状态是否碰撞
        // flag_got_start: 是否已经找到起点
        // flag_got_end: 是否已经找到终点
        if (occ && !last_occ)
        {
          if (same_occ_state_times > ENOUGH_INTERVAL || i == order_)
          {
            in_id = i - 1;
            flag_got_start = true;
          }
          same_occ_state_times = 0;
          flag_got_end_maybe = false; // terminate in advance
        }
        else if (!occ && last_occ)
        {
          out_id = i;
          flag_got_end_maybe = true;
          same_occ_state_times = 0;
        }
        else
        {
          ++same_occ_state_times;
        }

        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == (int)init_points.cols() - order_)))
        {
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ;

        if (flag_got_start && flag_got_end)
        {
          flag_got_start = false;
          flag_got_end = false;
          if (in_id >= 0 && out_id >= in_id && out_id < init_points.cols())
            segment_ids.push_back(std::pair<int, int>(in_id, out_id));
          else
            RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                        "Skip invalid collision segment: in_id=%d, out_id=%d", in_id, out_id);
        }
      }
    }

    /*** a star search ***/
    vector<vector<Eigen::Vector3d>> a_star_paths;
    // 遍历所有碰撞段 
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      //cout << "in=" << in.transpose() << " out=" << out.transpose() << endl;
      // 碰撞段的起点和终点
      Eigen::Vector3d in(init_points.col(segment_ids[i].first)), out(init_points.col(segment_ids[i].second));
      // A*搜索
      ASTAR_RET ret = a_star_->AstarSearch(grid_map_->getResolution(), in, out);
      // 搜索成功
      if (ret == ASTAR_RET::SUCCESS)
      {
        vector<Eigen::Vector3d> path = a_star_->getPath();
        if (path.size() < 2)
        {
          RCLCPP_WARN(rclcpp::get_logger("bspline_opt"), "A-star path has fewer than 2 points");
          return a_star_paths;
        }
        a_star_paths.push_back(path);
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger("bspline_opt"), "A-star failed; aborting optimization");
        return a_star_paths;
      }
    }

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    // 碰撞区域对应的 B-spline 优化控制点范围
    vector<std::pair<int, int>> bounds(segment_ids.size());
    for (size_t i = 0; i < segment_ids.size(); i++)
    {

      if (i == 0) // first segment
      {
        id_low_bound = order_;
        if (segment_ids.size() > 1)
        {
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          id_up_bound = init_points.cols() - order_ - 1;
        }
      }
      else if (i == segment_ids.size() - 1) // last segment, i != 0 here
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = init_points.cols() - order_ - 1;
      }
      else
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    // cout << "+++++++++" << endl;
    // for ( int j=0; j<bounds.size(); ++j )
    // {
    //   cout << bounds[j].first << "  " << bounds[j].second << endl;
    // }

    /*** Adjust segment length ***/
    vector<std::pair<int, int>> final_segment_ids(segment_ids.size());
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient thrust
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** Adjust segment length ***/
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      //cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
      if (num_points < minimum_points)
      {
        // 每侧增加的控制点数
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        final_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first ? segment_ids[i].first - add_points_each_side : bounds[i].first;

        final_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second ? segment_ids[i].second + add_points_each_side : bounds[i].second;
      }
      else
      {
        final_segment_ids[i].first = segment_ids[i].first;
        final_segment_ids[i].second = segment_ids[i].second;
      }

      //cout << "final:" << "i = " << i << " first = " << final_segment_ids[i].first << " second = " << final_segment_ids[i].second << endl;
    }

    /*** Assign data to each segment ***/
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      if (i >= a_star_paths.size() || a_star_paths[i].size() < 2)
      {
        RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                    "Skip invalid A-star path while assigning control point directions");
        continue;
      }

      // step 1
      for (int j = final_segment_ids[i].first; j <= final_segment_ids[i].second; ++j)
        cps_.flag_temp[j] = false;

      // step 2
      int got_intersection_id = -1;
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
      {
        Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
        int Astar_id = a_star_paths[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_paths[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
        while (Astar_id >= 0 && Astar_id < (int)a_star_paths[i].size())
        {
          last_Astar_id = Astar_id;

          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          if (Astar_id < 0 || Astar_id >= (int)a_star_paths[i].size())
            break;

          val = (a_star_paths[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
          {
            const double denom = ctrl_pts_law.dot(a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]);
            if (std::abs(denom) < 1e-8)
              break;
            // 交点
            intersection_point =
                a_star_paths[i][Astar_id] +
                ((a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(cps_.points.col(j) - a_star_paths[i][Astar_id]) / denom) // = t
                );

            //cout << "i=" << i << " j=" << j << " Astar_id=" << Astar_id << " last_Astar_id=" << last_Astar_id << " intersection_point = " << intersection_point.transpose() << endl;

            got_intersection_id = j;
            break;
          }
        }

        if (got_intersection_id >= 0)
        {
          cps_.flag_temp[j] = true;
          double length = (intersection_point - cps_.points.col(j)).norm();
          if (length > 1e-5)
          {
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
            {
              // 线性插值
              Eigen::Vector3d sample_pt = (a / length) * intersection_point + (1 - a / length) * cps_.points.col(j);
              double sample_yaw = estimateControlPointYaw(cps_.points, j);
              occ = grid_map_->getInflateOccupancy(sample_pt, sample_yaw);

              if (occ || a < grid_map_->getResolution())
              {
                if (occ)
                  a += grid_map_->getResolution();
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                break;
              }
            }
          }
        }
      }

      /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
      // 只有两个控制点时，需要特殊处理
      if (segment_ids[i].second - segment_ids[i].first == 1)
      {
        Eigen::Vector3d ctrl_pts_law(cps_.points.col(segment_ids[i].second) - cps_.points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector3d middle_point = (cps_.points.col(segment_ids[i].second) + cps_.points.col(segment_ids[i].first)) / 2;
        int Astar_id = a_star_paths[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        // 定义中垂
        double val = (a_star_paths[i][Astar_id] - middle_point).dot(ctrl_pts_law), last_val = val;
        while (Astar_id >= 0 && Astar_id < (int)a_star_paths[i].size())
        {
          last_Astar_id = Astar_id;

          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          if (Astar_id < 0 || Astar_id >= (int)a_star_paths[i].size())
            break;

          val = (a_star_paths[i][Astar_id] - middle_point).dot(ctrl_pts_law);

          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
          {
            const double denom = ctrl_pts_law.dot(a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]);
            if (std::abs(denom) < 1e-8)
              break;

            intersection_point =
                a_star_paths[i][Astar_id] +
                ((a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_paths[i][Astar_id]) / denom) // = t
                );

            if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
            {
              cps_.flag_temp[segment_ids[i].first] = true;
              cps_.base_point[segment_ids[i].first].push_back(cps_.points.col(segment_ids[i].first));
              cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

              got_intersection_id = segment_ids[i].first;
            }
            break;
          }
        }
      }

      //step 3
      if (got_intersection_id >= 0)
      {
        for (int j = got_intersection_id + 1; j <= final_segment_ids[i].second; ++j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
          }

        for (int j = got_intersection_id - 1; j >= final_segment_ids[i].first; --j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
          }
      }
      else
      {
        // Just ignore, it does not matter ^_^.
        // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      }
    }

    return a_star_paths;
  }

  int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    // cout << "k=" << k << endl;
    // cout << "opt->flag_continue_to_optimize_=" << opt->flag_continue_to_optimize_ << endl;
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRebound(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }

  double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRefine(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }

  bool BsplineOptimizer::pointInCorridor(int idx) const
  {
    return idx >= 0 && idx < static_cast<int>(cps_.in_corridor.size()) && cps_.in_corridor[idx];
  }

  bool BsplineOptimizer::corridorPointDodging(int idx) const
  {
    if (!pointInCorridor(idx) || idx >= static_cast<int>(cps_.direction.size()))
      return false;
    return !cps_.direction[idx].empty();
  }
  // 从 origin 出发，沿着 normal 指定的方向，在栅格地图里做一次射线扫描，寻找这个方向上遇到的第一个障碍体素，并把障碍体素中心坐标返回到 hit
  bool BsplineOptimizer::castToRawOccupancy(const Eigen::Vector3d &origin, const Eigen::Vector3d &normal,
                                            Eigen::Vector3d &hit) const
  {
    const double res = grid_map_->getResolution();
    if (res <= 1e-6 || corridor_max_range_ <= res)
      return false;
    // 射线终点
    const Eigen::Vector3d target = origin + normal * corridor_max_range_;
    RayCaster caster;
    if (!caster.setInput(origin / res, target / res))
      return false;

    Eigen::Vector3d voxel;
    bool skipped_start = false;
    const int max_steps = static_cast<int>(corridor_max_range_ / res) + 3;
    for (int step = 0; step < max_steps; ++step)
    {
      if (!caster.step(voxel))
        return false;

      const Eigen::Vector3d pos = (voxel + Eigen::Vector3d(0.5, 0.5, 0.5)) * res;
      if (!skipped_start)
      {
        skipped_start = true;
        continue;
      }
      if (!grid_map_->isInMap(pos))
        return false;
      if (grid_map_->getOccupancy(pos) != 1)
        continue;

      const double dist = (pos - origin).dot(normal);
      if (dist <= 1e-3)
        continue;

      hit = pos;
      return true;
    }
    return false;
  }
  // 对 B-spline 轨迹的控制点进行左右两侧的走廊检测，并判断哪些控制点可以启用走廊约束
  void BsplineOptimizer::updateCorridorAnchors(const char *stage)
  {
    // 初始化
    const int n = cps_.size;
    cps_.corridor_hit_left.assign(n, Eigen::Vector3d::Zero());
    cps_.corridor_hit_right.assign(n, Eigen::Vector3d::Zero());
    cps_.corridor_normal_left.assign(n, Eigen::Vector3d::Zero());
    cps_.corridor_normal_right.assign(n, Eigen::Vector3d::Zero());
    cps_.in_corridor.assign(n, 0); 

    if (!grid_map_)
    {
      RCLCPP_WARN(rclcpp::get_logger("bspline_opt"), "[corridor] %s no grid map, all points stay on dist0/fitness", stage);
      return;
    }
    if (corridor_width_ <= 0.0 || corridor_max_range_ <= 0.0)
    {
      RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                  "[corridor] %s disabled: width=%.3f max_range=%.3f", stage, corridor_width_, corridor_max_range_);
      return;
    }

    const double radius = std::max(0.0, grid_map_->getDoubleCylinderRadius());
    const double d_des = radius + corridor_margin_;
    // 首尾控制点不会参与这次走廊检测
    const int begin = std::min(order_, n);
    const int end = std::max(begin, n - order_);
    int corridor_count = 0;
    int miss_left = 0, miss_right = 0, too_wide = 0;

    RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                "[corridor] %s measure points [%d, %d) width_th=%.3f range=%.3f radius=%.3f margin=%.3f d_des=%.3f cps=%d",
                stage, begin, end, corridor_width_, corridor_max_range_, radius, corridor_margin_, d_des, n);

    for (int i = begin; i < end; ++i)
    {
      const Eigen::Vector3d q = cps_.points.col(i);
      const double yaw = estimateControlPointYaw(cps_.points, i);
      const Eigen::Vector3d left_n(-std::sin(yaw), std::cos(yaw), 0.0);
      const Eigen::Vector3d right_n = -left_n;

      Eigen::Vector3d hit_l, hit_r;
      const bool got_l = castToRawOccupancy(q, left_n, hit_l);
      const bool got_r = castToRawOccupancy(q, right_n, hit_r);
      const double d_l = got_l ? (hit_l - q).dot(left_n) : -1.0;
      const double d_r = got_r ? (hit_r - q).dot(right_n) : -1.0;

      const char *reason = "in_corridor";
      if (!got_l || !got_r)
      {
        if (!got_l)
          ++miss_left;
        if (!got_r)
          ++miss_right;
        reason = !got_l && !got_r ? "both_miss" : (!got_l ? "left_miss" : "right_miss");
      }
      else if (d_l + d_r >= corridor_width_)
      {
        ++too_wide;
        reason = "too_wide";
      }
      else
      {
        cps_.in_corridor[i] = 1;
        cps_.corridor_hit_left[i] = hit_l;
        cps_.corridor_hit_right[i] = hit_r;
        cps_.corridor_normal_left[i] = left_n;
        cps_.corridor_normal_right[i] = right_n;
        ++corridor_count;
      }

      RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                  "[corridor] %s i=%d pos=(%.3f, %.3f, %.3f) yaw=%.3f dL=%.3f dR=%.3f width=%.3f hitL=(%.3f, %.3f) hitR=(%.3f, %.3f) -> %s%s",
                  stage, i, q.x(), q.y(), q.z(), yaw, d_l, d_r, (got_l && got_r) ? d_l + d_r : -1.0,
                  got_l ? hit_l.x() : 0.0, got_l ? hit_l.y() : 0.0, got_r ? hit_r.x() : 0.0, got_r ? hit_r.y() : 0.0,
                  reason,
                  !cps_.in_corridor[i] ? ", keep dist0/fitness"
                  : corridorPointDodging(i) ? ", collision anchor, keep dist0/fitness"
                                            : ", skip dist0 and fitness");
    }

    RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                "[corridor] %s summary corridor=%d too_wide=%d miss_left=%d miss_right=%d checked=%d",
                stage, corridor_count, too_wide, miss_left, miss_right, end - begin);
  }

  void BsplineOptimizer::calcCorridorCost(const Eigen::MatrixXd &q, double &cost_center, double &cost_clearance,
                                          Eigen::MatrixXd &gradient_center, Eigen::MatrixXd &gradient_clearance) const
  {
    cost_center = 0.0;
    cost_clearance = 0.0;
    if (!grid_map_)
      return;

    const double d_des = std::max(0.0, grid_map_->getDoubleCylinderRadius()) + corridor_margin_;
    const int end_idx = std::min(static_cast<int>(q.cols()), cps_.size) - order_;
    for (int i = order_; i < end_idx; ++i)
    {
      // 已有碰撞锚点的通道点要绕障碍，不再往通道中线拉。
      if (!pointInCorridor(i) || corridorPointDodging(i))
        continue;

      const Eigen::Vector3d qi = q.col(i);
      const Eigen::Vector3d &n_l = cps_.corridor_normal_left[i];
      const Eigen::Vector3d &n_r = cps_.corridor_normal_right[i];
      const double d_l = (cps_.corridor_hit_left[i] - qi).dot(n_l);
      const double d_r = (cps_.corridor_hit_right[i] - qi).dot(n_r);
      const double diff = d_l - d_r;
      cost_center += diff * diff;
      gradient_center.col(i) += 2.0 * diff * (-n_l + n_r);

      const double err_l = d_des - d_l;
      if (err_l > 0.0)
      {
        cost_clearance += err_l * err_l;
        gradient_clearance.col(i) += 2.0 * err_l * n_l;
      }
      const double err_r = d_des - d_r;
      if (err_r > 0.0)
      {
        cost_clearance += err_r * err_r;
        gradient_clearance.col(i) += 2.0 * err_r * n_r;
      }
    }
  }

  void BsplineOptimizer::logCorridorState(const char *stage) const
  {
    if (!grid_map_)
      return;

    const double d_des = std::max(0.0, grid_map_->getDoubleCylinderRadius()) + corridor_margin_;
    int count = 0;
    for (int i = 0; i < cps_.size; ++i)
    {
      if (!pointInCorridor(i))
        continue;
      const Eigen::Vector3d q = cps_.points.col(i);
      const double d_l = (cps_.corridor_hit_left[i] - q).dot(cps_.corridor_normal_left[i]);
      const double d_r = (cps_.corridor_hit_right[i] - q).dot(cps_.corridor_normal_right[i]);
      RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                  "[corridor] %s result i=%d pos=(%.3f, %.3f, %.3f) dL=%.3f dR=%.3f |dL-dR|=%.3f marginL=%.3f marginR=%.3f d_des=%.3f",
                  stage, i, q.x(), q.y(), q.z(), d_l, d_r, std::abs(d_l - d_r), d_l - d_des, d_r - d_des, d_des);
      ++count;
    }
    RCLCPP_INFO(rclcpp::get_logger("bspline_opt"), "[corridor] %s result points=%d", stage, count);
  }

  void BsplineOptimizer::logCostBreakdown(const char *stage, double f_combine, double f_smooth, double w_smooth,
                                          double f_second, double w_second, const char *second_name,
                                          double f_feas, double w_feas, double f_cor, double w_cor,
                                          double f_clr, double w_clr, const Eigen::MatrixXd &g_smooth,
                                          const Eigen::MatrixXd &g_center, const Eigen::MatrixXd &g_clearance) const
  {
    if (iter_num_ % 25 != 0)
      return;

    double corridor_grad = 0.0;
    double smooth_grad = 0.0;
    int corridor_points = 0;
    const int end_idx = std::min(static_cast<int>(g_smooth.cols()), cps_.size) - order_;
    for (int i = order_; i < end_idx; ++i)
    {
      if (!pointInCorridor(i))
        continue;
      ++corridor_points;
      corridor_grad += (w_cor * g_center.col(i) + w_clr * g_clearance.col(i)).head<2>().norm();
      smooth_grad += (w_smooth * g_smooth.col(i)).head<2>().norm();
    }

    RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                "[corridor] %s cost iter=%d f=%.4f smooth=%.4f(w=%.4f) %s=%.4f(w=%.4f) feas=%.4f(w=%.4f) center=%.4f(w=%.4f) clear=%.4f(w=%.4f) corridor_pts=%d grad_xy cor=%.4f smooth=%.4f",
                stage, iter_num_, f_combine, f_smooth, w_smooth, second_name, f_second, w_second, f_feas, w_feas,
                f_cor, w_cor, f_clr, w_clr, corridor_points, corridor_grad, smooth_grad);
  }

  void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                 Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_;
    double demarcation = cps_.clearance;
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

    force_stop_type_ = DONT_STOP;
    if (iter_num > 3 && smoothness_cost / (cps_.size - 2 * order_) < 0.1) // 0.1 is an experimental value that indicates the trajectory is smooth enough.
    {
      check_collision_and_rebound();
    }

    /*** calculate distance cost and gradient ***/
    for (auto i = order_; i < end_idx; ++i)
    {
      // 空通道才关掉 dist0。通道里挡住轨迹的障碍会留下碰撞锚点，这些点继续外推。
      if (pointInCorridor(i) && !corridorPointDodging(i))
        continue;

      for (size_t j = 0; j < cps_.direction[i].size(); ++j)
      {
        double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
        double dist_err = cps_.clearance - dist;
        Eigen::Vector3d dist_grad = cps_.direction[i][j];

        if (dist_err < 0)
        {
          /* do nothing */
        }
        else if (dist_err < demarcation)
        {
          cost += pow(dist_err, 3);
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
        }
        else
        {
          cost += a * dist_err * dist_err + b * dist_err + c;
          gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
        }
      }
    }
  }

  void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {

    cost = 0.0;

    int end_idx = q.cols() - order_;

    // def: f = |x*v|^2/a^2 + |x×v|^2/b^2
    double a2 = 25, b2 = 1;
    for (auto i = order_ - 1; i < end_idx + 1; ++i)
    {
      if (pointInCorridor(i) && !corridorPointDodging(i))
        continue;

      Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
      Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized();

      double xdotv = x.dot(v);
      Eigen::Vector3d xcrossv = x.cross(v);

      double f = pow((xdotv), 2) / a2 + pow(xcrossv.norm(), 2) / b2;
      cost += f;

      Eigen::Matrix3d m;
      m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
      Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

      gradient.col(i - 1) += df_dx / 6;
      gradient.col(i) += 4 * df_dx / 6;
      gradient.col(i + 1) += df_dx / 6;
    }
  }

  void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
  {

    cost = 0.0;

    if (falg_use_jerk)
    {
      Eigen::Vector3d jerk, temp_j;

      for (int i = 0; i < q.cols() - 3; i++)
      {
        /* evaluate jerk */
        jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
        cost += jerk.squaredNorm();
        temp_j = 2.0 * jerk;
        /* jerk gradient */
        gradient.col(i + 0) += -temp_j;
        gradient.col(i + 1) += 3.0 * temp_j;
        gradient.col(i + 2) += -3.0 * temp_j;
        gradient.col(i + 3) += temp_j;
      }
    }
    else
    {
      Eigen::Vector3d acc, temp_acc;

      for (int i = 0; i < q.cols() - 2; i++)
      {
        /* evaluate acc */
        acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
        cost += acc.squaredNorm();
        temp_acc = 2.0 * acc;
        /* acc gradient */
        gradient.col(i + 0) += temp_acc;
        gradient.col(i + 1) += -2.0 * temp_acc;
        gradient.col(i + 2) += temp_acc;
      }
    }
  }

  void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                             Eigen::MatrixXd &gradient)
  {

    //#define SECOND_DERIVATIVE_CONTINOUS

#ifdef SECOND_DERIVATIVE_CONTINOUS

    cost = 0.0;
    double demarcation = 1.0; // 1m/s, 1m/s/s
    double ar = 3 * demarcation, br = -3 * pow(demarcation, 2), cr = pow(demarcation, 3);
    double al = ar, bl = -br, cl = cr;

    /* abbreviation */
    double ts, ts_inv2, ts_inv3;
    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;
    ts_inv3 = 1 / ts / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_ + demarcation)
        {
          double diff = vi(j) - max_vel_;
          cost += (ar * diff * diff + br * diff + cr) * ts_inv3; // multiply ts_inv3 to make vel and acc has similar magnitude

          double grad = (2.0 * ar * diff + br) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) > max_vel_)
        {
          double diff = vi(j) - max_vel_;
          cost += pow(diff, 3) * ts_inv3;
          ;

          double grad = 3 * diff * diff / ts * ts_inv3;
          ;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -(max_vel_ + demarcation))
        {
          double diff = vi(j) + max_vel_;
          cost += (al * diff * diff + bl * diff + cl) * ts_inv3;

          double grad = (2.0 * al * diff + bl) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -max_vel_)
        {
          double diff = vi(j) + max_vel_;
          cost += -pow(diff, 3) * ts_inv3;

          double grad = -3 * diff * diff / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_ + demarcation)
        {
          double diff = ai(j) - max_acc_;
          cost += ar * diff * diff + br * diff + cr;

          double grad = (2.0 * ar * diff + br) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) > max_acc_)
        {
          double diff = ai(j) - max_acc_;
          cost += pow(diff, 3);

          double grad = 3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -(max_acc_ + demarcation))
        {
          double diff = ai(j) + max_acc_;
          cost += al * diff * diff + bl * diff + cl;

          double grad = (2.0 * al * diff + bl) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -max_acc_)
        {
          double diff = ai(j) + max_acc_;
          cost += -pow(diff, 3);

          double grad = -3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

#else

    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      //cout << "temp_v * vi=" ;
      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_)
        {
          // cout << "fuck VEL" << endl;
          // cout << vi(j) << endl;
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude

          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      //cout << "temp_a * ai=" ;
      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_)
        {
          // cout << "fuck ACC" << endl;
          // cout << ai(j) << endl;
          cost += pow(ai(j) - max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        else if (ai(j) < -max_acc_)
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
      //cout << endl;
    }

#endif
  }

  bool BsplineOptimizer::check_collision_and_rebound(void)
  {

    int end_idx = cps_.size - order_;

    /*** Check and segment the initial trajectory according to obstacles ***/
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    int i_end = end_idx - (end_idx - order_) / 3;
    for (int i = order_ - 1; i <= i_end; ++i)
    {

      bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i), estimateControlPointYaw(cps_.points, i));

      /*** check if the new collision will be valid ***/
      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          cout.precision(2);
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // current point is outside all the collision_points.
          {
            occ = false; // Not really takes effect, just for better hunman understanding.
            break;
          }
        }
      }

      if (occ)
      {
        flag_new_obs_valid = true;

        int j;
        for (j = i - 1; j >= 0; --j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j), estimateControlPointYaw(cps_.points, j));
          if (!occ)
          {
            in_id = j;
            break;
          }
        }
        if (j < 0) // fail to get the obs free point
        {
          RCLCPP_ERROR(rclcpp::get_logger("bspline_opt"), "The robot is inside an obstacle");
          in_id = 0;
        }

        for (j = i + 1; j < cps_.size; ++j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j), estimateControlPointYaw(cps_.points, j));

          if (!occ)
          {
            out_id = j;
            break;
          }
        }
        if (j >= cps_.size) // fail to get the obs free point
        {
          RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                      "Trajectory terminal point is in an obstacle; skip this plan");

          force_stop_type_ = STOP_FOR_ERROR;
          return false;
        }

        i = j + 1;

        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
      }
    }

    if (flag_new_obs_valid)
    {
      vector<vector<Eigen::Vector3d>> a_star_paths;
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        /*** a star search ***/
        Eigen::Vector3d in(cps_.points.col(segment_ids[i].first)), out(cps_.points.col(segment_ids[i].second));
        ASTAR_RET ret = a_star_->AstarSearch(grid_map_->getResolution(), in, out);
        if (ret == ASTAR_RET::SUCCESS)
        {
          vector<Eigen::Vector3d> path = a_star_->getPath();
          if (path.size() < 2)
          {
            RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                        "A-star path has fewer than 2 points; drop collision segment");
            segment_ids.erase(segment_ids.begin() + i);
            i--;
            continue;
          }
          a_star_paths.push_back(path);
        }
        else if (ret == ASTAR_RET::SEARCH_ERR && i + 1 < segment_ids.size())
        {
          segment_ids[i].second = segment_ids[i + 1].second;
          segment_ids.erase(segment_ids.begin() + i + 1);
          --i;
          RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                      "A-star failed on a collision segment; merge it with the next segment");
        }
        else
        {
          RCLCPP_ERROR(rclcpp::get_logger("bspline_opt"), "A-star error");
          segment_ids.erase(segment_ids.begin() + i);
          i--;
        }
      }

      /*** Assign parameters to each segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        if (i >= a_star_paths.size() || a_star_paths[i].size() < 2)
        {
          RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                      "Skip invalid A-star path while assigning rebound directions");
          continue;
        }

        // step 1
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        // step 2
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          int Astar_id = a_star_paths[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_paths[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
          while (Astar_id >= 0 && Astar_id < (int)a_star_paths[i].size())
          {
            last_Astar_id = Astar_id;

            if (val >= 0)
              --Astar_id;
            else
              ++Astar_id;

            if (Astar_id < 0 || Astar_id >= (int)a_star_paths[i].size())
              break;

            val = (a_star_paths[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

            // cout << val << endl;

            if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
            {
              const double denom = ctrl_pts_law.dot(a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]);
              if (std::abs(denom) < 1e-8)
                break;

              intersection_point =
                  a_star_paths[i][Astar_id] +
                  ((a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(j) - a_star_paths[i][Astar_id]) / denom) // = t
                  );

              got_intersection_id = j;
              break;
            }
          }

          if (got_intersection_id >= 0)
          {
            cps_.flag_temp[j] = true;
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                Eigen::Vector3d sample_pt = (a / length) * intersection_point + (1 - a / length) * cps_.points.col(j);
                double sample_yaw = estimateControlPointYaw(cps_.points, j);
                bool occ = grid_map_->getInflateOccupancy(sample_pt, sample_yaw);

                if (occ || a < grid_map_->getResolution())
                {
                  if (occ)
                    a += grid_map_->getResolution();
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                  break;
                }
              }
            }
            else
            {
              got_intersection_id = -1;
            }
          }
        }

        //step 3
        if (got_intersection_id >= 0)
        {
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }

          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
        else
          RCLCPP_WARN(rclcpp::get_logger("bspline_opt"), "Failed to generate rebound direction");
      }

      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    return false;
  }

  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
  {
    setBsplineInterval(ts);

    bool flag_success = rebound_optimize();

    optimal_points = cps_.points;

    return flag_success;
  }

  bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points)
  {

    setControlPoints(init_points);
    setBsplineInterval(ts);

    bool flag_success = refine_optimize();

    optimal_points = cps_.points;

    return flag_success;
  }

  bool BsplineOptimizer::rebound_optimize()
  {
    iter_num_ = 0;
    // 固定轨迹端点，只优化中间的控制点
    int start_id = order_;
    int end_id = this->cps_.size - order_;
    // 优化变量数量
    variable_num_ = 3 * (end_id - start_id);
    double final_cost;

    auto t0 = std::chrono::steady_clock::now();
    auto t1 = t0;
    auto t2 = t0;
    int restart_nums = 0, rebound_times = 0;
    bool flag_force_return, flag_occ, success;
    new_lambda2_ = lambda2_;
    // 最大restart次数
    constexpr int MAX_RESART_NUMS_SET = 3;
    do
    {
      /* ---------- prepare ---------- */
      min_cost_ = std::numeric_limits<double>::max();
      iter_num_ = 0;
      flag_force_return = false;
      flag_occ = false;
      success = false;

      updateCorridorAnchors(restart_nums == 0 ? "rebound" : "rebound-restart");
      if (restart_nums > 0)
      {
        RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                    "[corridor] rebound restart=%d lambda_collision=%.3f; corridor points still skip dist0",
                    restart_nums, new_lambda2_);
      }

      double q[variable_num_];
      memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.01;

      /* ---------- optimize ---------- */
      t1 = std::chrono::steady_clock::now();
      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
      t2 = std::chrono::steady_clock::now();
      double time_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
      double total_time_ms = std::chrono::duration<double, std::milli>(t2 - t0).count();

      /* ---------- success temporary, check collision again ---------- */
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        //ROS_WARN("Solver error in planning!, return = %s", lbfgs::lbfgs_strerror(result));
        flag_force_return = false;

        UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
        double tm, tmp;
        traj.getTimeSpan(tm, tmp);
        double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution());
        for (double t = tm; t < tmp * 2 / 3; t += t_step) // Only check the closest 2/3 partition of the whole trajectory.
        {
          Eigen::Vector3d pos = traj.evaluateDeBoorT(t);
          Eigen::Vector3d pos_next = traj.evaluateDeBoorT(std::min(t + t_step, tmp));
          flag_occ = grid_map_->getInflateOccupancy(pos, estimateSegmentYaw(pos, pos_next));
          if (flag_occ)
          {
            //cout << "hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

            if (t <= bspline_interval_) // First 3 control points in obstacles!
            {
              cout << cps_.points.col(1).transpose() << "\n"
                   << cps_.points.col(2).transpose() << "\n"
                   << cps_.points.col(3).transpose() << "\n"
                   << cps_.points.col(4).transpose() << endl;
              RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                          "First three control points are in obstacles; t=%f", t);
              return false;
            }

            break;
          }
        }

        if (!flag_occ)
        {
          printf("\033[32miter(+1)=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
          logCorridorState("rebound-ok");
          success = true;
        }
        else // restart
        {
          restart_nums++;
          logCorridorState("rebound-still-colliding");
          int corridor_hits = 0;
          for (int i = order_; i < cps_.size - order_; ++i)
          {
            if (!pointInCorridor(i))
              continue;
            const int occ = grid_map_->getInflateOccupancy(cps_.points.col(i), estimateControlPointYaw(cps_.points, i));
            if (occ == 0)
              continue;
            ++corridor_hits;
            RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                        "[corridor] rebound colliding control point i=%d pos=(%.3f, %.3f, %.3f) inflate_occ=%d",
                        i, cps_.points(0, i), cps_.points(1, i), cps_.points(2, i), occ);
          }
          RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                      "[corridor] rebound still colliding, restart=%d corridor_points_in_inflation=%d",
                      restart_nums, corridor_hits);
          initControlPoints(cps_.points, false);
          new_lambda2_ *= 2;

          printf("\033[32miter(+1)=%d,time(ms)=%5.3f,keep optimizing\n\033[0m", iter_num_, time_ms);
        }
      }
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        flag_force_return = true;
        rebound_times++;
        logCorridorState("rebound-early-exit");
        RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                    "[corridor] rebound early-exit times=%d iter=%d force_stop=%d",
                    rebound_times, iter_num_, static_cast<int>(force_stop_type_));
        cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",rebound." << endl;
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                    "Solver error. Return=%d, %s; skip this plan", result, lbfgs::lbfgs_strerror(result));
        // while (ros::ok());
      }

    } while ((flag_occ && restart_nums < MAX_RESART_NUMS_SET) ||
             (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

    RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                "[corridor] rebound finished success=%d restarts=%d early_exits=%d",
                success ? 1 : 0, restart_nums, rebound_times);
    return success;
  }

  bool BsplineOptimizer::refine_optimize()
  {
    iter_num_ = 0;
    int start_id = order_;
    int end_id = this->cps_.points.cols() - order_;
    variable_num_ = 3 * (end_id - start_id);

    double q[variable_num_];
    double final_cost;

    memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

    double origin_lambda4 = lambda4_;
    bool flag_safe = true;
    int iter_count = 0;
    do
    {
      updateCorridorAnchors("refine");
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.001;

      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        //pass
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger("bspline_opt"),
                     "Solver error while refining: return=%d, %s", result, lbfgs::lbfgs_strerror(result));
      }

      UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
      double tm, tmp;
      traj.getTimeSpan(tm, tmp);
      double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution()); // Step size is defined as the maximum size that can passes through every grid.
      for (double t = tm; t < tmp * 2 / 3; t += t_step)
      {
        Eigen::Vector3d pos = traj.evaluateDeBoorT(t);
        Eigen::Vector3d pos_next = traj.evaluateDeBoorT(std::min(t + t_step, tmp));
        if (grid_map_->getInflateOccupancy(pos, estimateSegmentYaw(pos, pos_next)))
        {
          // cout << "Refined traj hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

          Eigen::MatrixXd ref_pts(ref_pts_.size(), 3);
          for (size_t i = 0; i < ref_pts_.size(); i++)
          {
            ref_pts.row(i) = ref_pts_[i].transpose();
          }

          flag_safe = false;
          break;
        }
      }

      logCorridorState(flag_safe ? "refine-ok" : "refine-still-colliding");
      if (!flag_safe)
      {
        RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                    "[corridor] refine trajectory still hits inflation, lambda_fitness %.3f -> %.3f",
                    lambda4_, lambda4_ * 2.0);
        lambda4_ *= 2;
      }

      iter_count++;
    } while (!flag_safe && iter_count <= 0);

    lambda4_ = origin_lambda4;

    //cout << "iter_num_=" << iter_num_ << endl;

    return flag_safe;
  }

  void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
  {

    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_distance, f_feasibility, f_center, f_clearance;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_center = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_clearance = Eigen::MatrixXd::Zero(3, cps_.size);

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);
    calcCorridorCost(cps_.points, f_center, f_clearance, g_center, g_clearance);

    f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility +
                lambda_corridor_ * f_center + lambda_clearance_ * f_clearance;
    logCostBreakdown("rebound", f_combine, f_smoothness, lambda1_, f_distance, new_lambda2_, "dist",
                     f_feasibility, lambda3_, f_center, lambda_corridor_, f_clearance, lambda_clearance_,
                     g_smoothness, g_center, g_clearance);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance +
                              lambda3_ * g_feasibility + lambda_corridor_ * g_center +
                              lambda_clearance_ * g_clearance;
    grad_3D.row(2).setZero();
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

  void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
  {

    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_fitness, f_feasibility, f_center, f_clearance;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_center = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_clearance = Eigen::MatrixXd::Zero(3, cps_.points.cols());

    //time_satrt = ros::Time::now();

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcFitnessCost(cps_.points, f_fitness, g_fitness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);
    calcCorridorCost(cps_.points, f_center, f_clearance, g_center, g_clearance);

    /* ---------- convert to solver format...---------- */
    f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility +
                lambda_corridor_ * f_center + lambda_clearance_ * f_clearance;
    logCostBreakdown("refine", f_combine, f_smoothness, lambda1_, f_fitness, lambda4_, "fitness",
                     f_feasibility, lambda3_, f_center, lambda_corridor_, f_clearance, lambda_clearance_,
                     g_smoothness, g_center, g_clearance);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + lambda4_ * g_fitness +
                              lambda3_ * g_feasibility + lambda_corridor_ * g_center +
                              lambda_clearance_ * g_clearance;
    grad_3D.row(2).setZero();
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

} // namespace scan_planner
