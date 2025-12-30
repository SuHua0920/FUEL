#include <Eigen/Eigen>

#include <vector>

/**
 * @brief 获取可选择路径点中,以当前方向能规划的最远点
 * 
 * @param path 路径点的向量，包含多个3D坐标点
 * @param x1 线段的起始点
 * @param x2 线段的结束点
 * @return 返回路径中距离起始点和给定方向最远的点
 */
Eigen::Vector3d getFarPoint(const std::vector<Eigen::Vector3d>& path, Eigen::Vector3d x1,
                            Eigen::Vector3d x2) {
  double max_dist = -1000;
  Eigen::Vector3d vl = (x2 - x1).normalized();
  Eigen::Vector3d far_pt;

  for (int i = 1; i < path.size() - 1; ++i) {
    double dist = ((path[i] - x1).cross(vl)).norm();
    if (dist > max_dist) {
      max_dist = dist;
      far_pt = path[i];
    }
  }

  return far_pt;
}
