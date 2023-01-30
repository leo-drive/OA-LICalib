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

#include <trajectory/trajectory_viewer.h>
namespace liso {

namespace publisher {
rclcpp::Publisher<oa_licalib_msgs::msg::PoseArray>::SharedPtr pub_trajectory_raw_;
rclcpp::Publisher<oa_licalib_msgs::msg::PoseArray>::SharedPtr pub_trajectory_est_;
rclcpp::Publisher<oa_licalib_msgs::msg::ImuArray>::SharedPtr pub_imu_raw_array_;
rclcpp::Publisher<oa_licalib_msgs::msg::ImuArray>::SharedPtr pub_imu_est_array_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_target_cloud_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_source_cloud_;

rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_spline_trajectory_;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_lidar_trajectory_;

void SetPublisher(rclcpp::Node::SharedPtr  node) {
  /// Vicon data
  pub_trajectory_raw_ =  this->create_publisher<oa_licalib_msgs::msg::PoseArray>("/path_raw", 10);
  pub_trajectory_est_ =  this->create_publisher<oa_licalib_msgs::msg::PoseArray>("/path_est", 10);
  /// IMU fitting results
  pub_imu_raw_array_ =  this->create_publisher<oa_licalib_msgs::msg::ImuArray>("/imu_raw_array", 10);
  pub_imu_est_array_ =  this->create_publisher<oa_licalib_msgs::msg::ImuArray>("/imu_est_array", 10);
  /// lidar matching results
  pub_target_cloud_ =
          this->create_publisher<sensor_msgs::msg::PointCloud2>("/target_cloud", 10);
  pub_source_cloud_ =
          this->create_publisher<sensor_msgs::msg::PointCloud2>("/source_cloud", 10);

  /// spline trajectory
  pub_spline_trajectory_ =
          this->create_publisher<nav_msgs::msg::Path>("/spline_trajectory", 10);

  /// spline trajectory
  pub_lidar_trajectory_ = this->create_publisher<nav_msgs::msg::Path>("/lidar_trajectory", 10);
}

}  // namespace publisher

}  // namespace liso
