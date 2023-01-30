/*
 * OA-LICalib:
 * Observability-Aware Intrinsic and Extrinsic Calibration of LiDAR-IMU Systems
 *
 * Copyright (C) 2022 Jiajun Lv
 * Copyright (C) 2022 Xingxing Zuo
 * Copyright (C) 2022 Kewei Hu
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once
#include <rclcpp/rclcpp.hpp>
//#include <geometry_msgs/msg/PoseWithCovarianceStamped.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Eigen>
#include <memory>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <calib/ndt_registration.h>
#include <sensor_data/imu_data.h>
#include <sensor_data/lidar_feature.h>
#include <utils/math_utils.h>
#include <utils/eigen_utils.hpp>

namespace liso {

class LIDARLocalization : public rclcpp::Node  {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<LIDARLocalization> Ptr;

  explicit LIDARLocalization(
      double ndt_resolution = 0.5, double ndt_key_frame_downsample = 0.1,
      double map_downsample_size = 0.5,
      const Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity()) : Node("LIDARLocalization"){


  }

  void FeedScan(
      const LiDARFeature& cur_scan,
      const Eigen::Matrix4d& pose_predict = Eigen::Matrix4d::Identity(),
      bool pridict_is_relative = true);

  void SetPriorMap(std::string map_pcd_path);

  void SetPriorMap(const LiDARFeature& map_cloud);

  void SetInitPose(const Eigen::Matrix4d& new_init_pose) {
    init_pose_ = new_init_pose;
  }

  void CaculateGlobalMapAndOdom(
      const std::map<double, LiDARFeature>& scan_data);

  const PosCloud::Ptr& GetNewMapCloud() const { return new_map_.full_features; }

  void SaveGlobalMap(std::string path) {
    std::cout << "save ndt locator map to " << path
              << "; size : " << new_map_.full_features->size() << std::endl;
    pcl::io::savePCDFileBinaryCompressed(path, *new_map_.full_features);
  }

  void PublishCloudAndOdom(const PosCloud::Ptr& cur_scan = nullptr);

  const Eigen::aligned_vector<OdomData>& get_odom_data() const {
    return odom_data_;
  }

  void ClearOdomData();

  void OdomToTUMTxt(std::string file_path, double relative_start_time,
                    double relative_end_time) const {
    std::string file_name = "/locator-odom-" + std::to_string(relative_start_time) +
                            "-" + std::to_string(relative_end_time) + ".txt";
    std::string traj_path = file_path + file_name;

    std::ofstream outfile;
    outfile.open(traj_path);

    for (const auto& v : odom_data_) {
      double relative_bag_time = v.timestamp + relative_start_time;
      Eigen::Vector3d p = v.pose.block<3, 1>(0, 3);
      Eigen::Quaterniond q = Eigen::Quaterniond(v.pose.block<3, 3>(0, 0));

      outfile.precision(9);
      outfile << relative_bag_time << " ";
      outfile.precision(5);
      outfile << p(0) << " " << p(1) << " " << p(2) << " " << q.x() << " "
              << q.y() << " " << q.z() << " " << q.w() << "\n";
    }
    outfile.close();
    std::cout << "Save ndt odom at " << traj_path << std::endl;
  }

 private:
  void RegisterPubSub();

  bool CheckKeyScan(const OdomData& odom_data);

  inline double NormalizeAngle(double ang_degree) {
    if (ang_degree > 180) ang_degree -= 360;

    if (ang_degree < -180) ang_degree += 360;
    return ang_degree;
  }

 private:
//  ros::NodeHandle nh_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_cloud_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_laser_odometry_;
  Eigen::Matrix4d init_pose_;

  double ndt_resolution_;
  double ndt_key_frame_downsample_;

  NDTRegistration::Ptr ndt_registration_;

  LiDARFeature prior_map_;
  sensor_msgs::msg::PointCloud2 map_msg_;
  pcl::VoxelGrid<PosPoint> map_filter_;

  pcl::VoxelGrid<PosPoint> cloud_filter_;

  LiDARFeature new_map_;

  std::vector<size_t> key_frame_index_;

  Eigen::aligned_vector<OdomData> odom_data_;
};

}  // namespace liso
