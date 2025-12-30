#ifndef _EXPL_DATA_H_
#define _EXPL_DATA_H_

#include <Eigen/Eigen>
#include <bspline/Bspline.h>
#include <vector>

using Eigen::Vector3d;
using std::vector;

namespace fast_planner {

//存储有限状态机的运行数据
struct FSMData {
  // FSM data
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 触发标志、里程计状态、静态状态等标识变量
  bool trigger_, have_odom_, static_state_;
  vector<string> state_str_;

  // 存储位置、速度、姿态等里程计信息
  Eigen::Vector3d odom_pos_, odom_vel_; // odometry state
  Eigen::Quaterniond odom_orient_;
  double odom_yaw_;

  // 记录起始状态的位置、速度、加速度, yaw角等参数
  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_; // start state
  //存储起始位置的容器，用于保存搜索算法中所有可能的起始位置坐标
  vector<Eigen::Vector3d> start_poss;
  //最新的轨迹数据
  bspline::Bspline newest_traj_;
};

// 用于控制状态机的决策和规划行为
struct FSMParam {
  double replan_thresh1_;
  double replan_thresh2_;
  double replan_thresh3_;
  double replan_time_; // second
};

// 存储探索任务中的各种数据
struct ExplorationData {

  // 前沿点集合
  vector<vector<Vector3d>> frontiers_;
  // 消失前沿点集合
  vector<vector<Vector3d>> dead_frontiers_;

  // 前沿边界框
  vector<pair<Vector3d, Vector3d>> frontier_boxes_;
  // 采样点、、、、等探索相关数据，以及和路径规划信息
  vector<Vector3d> points_;
  //平均点
  vector<Vector3d> averages_;
  //视角
  vector<Vector3d> views_;
  //偏航角
  vector<double> yaws_;
  //全局路径
  vector<Vector3d> global_tour_;
  //精细化处理后的点云数据
  vector<int> refined_ids_;
  vector<vector<Vector3d>> n_points_;
  // 未细化的点云数据
  vector<Vector3d> unrefined_points_;
  // 细化后的点云数据
  vector<Vector3d> refined_points_;
  // 视角信息
  vector<Vector3d> refined_views_; // points + dir(yaw)
  vector<Vector3d> refined_views1_, refined_views2_;
  vector<Vector3d> refined_tour_;

  Vector3d next_goal_;
  vector<Vector3d> path_next_goal_;

  // viewpoint planning
  // vector<Vector4d> views_;
  vector<Vector3d> views_vis1_, views_vis2_;
  vector<Vector3d> centers_, scales_;
};

//探索算法的参数配置
struct ExplorationParam {
  // 局部优化开关
  bool refine_local_;
  // 精细化数量
  int refined_num_;
  // 精细化半径
  double refined_radius_;
  // 俯视图数量
  int top_view_num_;
  // 最大衰减
  double max_decay_;
  // TSP求解器资源目录
  string tsp_dir_; // resource dir of tsp solver
  // 时间松弛参数
  double relax_time_;
};

} // namespace fast_planner

#endif