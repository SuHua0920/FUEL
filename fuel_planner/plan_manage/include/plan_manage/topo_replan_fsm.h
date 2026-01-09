#ifndef _TOPO_REPLAN_FSM_H_
#define _TOPO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline/Bspline.h>
#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace fast_planner {
// 在线重规划系统类，用于控制规划流程， 启动、执行和停止
class TopoReplanFSM {
private:
  /* ---------- flag ---------- */
  //执行状态机
  enum FSM_EXEC_STATE {
    INIT,         //初始化状态
    WAIT_TARGET,  //等待目标状态
    GEN_NEW_TRAJ, //生成新轨迹状态
    REPLAN_TRAJ,  //重规划状态
    EXEC_TRAJ,    //执行状态
    REPLAN_NEW    //重新规划状态
  };

  // 目标点类型
  enum TARGET_TYPE {
    MANUAL_TARGET = 1, //手动目标点rviz
    PRESET_TARGET = 2, //预设目标点
    REFENCE_PATH = 3   //参考路径
  };

  /* planning utils */
  
  FastPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;

  /* parameters */
  int target_type_; // 1 mannual select, 2 hard code
  double replan_distance_threshold_, replan_time_threshold_;
  double waypoints_[50][3];
  int waypoint_num_;
  bool act_map_;

  /* planning data */
  bool trigger_, have_target_, have_odom_, collide_;
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_, odom_vel_; // odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_; // start state
  Eigen::Vector3d target_point_, end_vel_;                       // target state
  int current_wp_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber waypoint_sub_, odom_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  /* helper functions */
  bool callSearchAndOptimization();   // front-end and back-end method
  bool callTopologicalTraj(int step); // topo path guided gradient-based
                                      // optimization; 1: new, 2: replan
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  void printFSMExecState();

  /* ROS functions */
  void execFSMCallback(const ros::TimerEvent &e);
  void checkCollisionCallback(const ros::TimerEvent &e);
  void frontierCallback(const ros::TimerEvent &e);
  void waypointCallback(const nav_msgs::PathConstPtr &msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);

public:
  TopoReplanFSM(/* args */) {}
  ~TopoReplanFSM() {}

  void init(ros::NodeHandle &nh);

  // benchmark
  vector<double> replan_time_;
  vector<double> replan_time2_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace fast_planner

#endif