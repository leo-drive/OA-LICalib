//
// Created by bzeren on 11.08.2023.
//

#pragma once

#include <angles/angles.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>

#include <sensor_data/lidar_feature.h>
#include <sensor_msgs/PointCloud2.h>

namespace liso {

enum LivoxRingNo {
    HAP = 6,
};

class LivoxLiDAR {
public:
    typedef std::shared_ptr<LivoxLiDAR> Ptr;

    LivoxLiDAR(double ring_no)
    : livox_ring_No_(ring_no), num_firing_(4000) {}

    void get_organized_and_raw_cloud(
            const sensor_msgs::PointCloud2::ConstPtr &lidarMsg,
                                        LiDARFeature &output) {
        LivoxPointCloud pc_in;
        pcl::fromROSMsg(*lidarMsg, pc_in);

        int ring_number = int(livox_ring_No_);
        int ring_step = pc_in.height / ring_number;

        assert(ring_step >= 1 && "LivoxRingNo too large");

        double timebase = lidarMsg->header.stamp.toSec();
        output.timestamp = timebase;

        /// point cloud
        output.full_features->clear();
        output.full_features->height = livox_ring_No_;
        output.full_features->width = num_firing_;
        output.full_features->is_dense = false;
        output.full_features->resize(output.full_features->height *
                                     output.full_features->width);

        /// raw_data
        output.raw_data->height = livox_ring_No_;
        output.raw_data->width = num_firing_;
        output.raw_data->is_dense = false;
        output.raw_data->resize(output.raw_data->height * output.raw_data->width);

        PosPoint NanPoint;
        NanPoint.x = NAN;
        NanPoint.y = NAN;
        NanPoint.z = NAN;
        NanPoint.timestamp = timebase;

        int num_points = livox_ring_No_ * num_firing_;
        for (int k = 0; k < num_points; k++) {
            output.full_features->points[k] = NanPoint;
            output.raw_data->points[k] = NanPoint;
        }

        //TODO: burda
        std::cout << "----1--1-1-1-1--1-1-1-1-1--1-1-1-1--1" << std::endl;
        for (int h = 0; h < livox_ring_No_; h++) {
            for (int w = 0; w < num_firing_; w++)
            {
                const auto &src = pc_in.at(h*num_firing_+w);

                double depth = sqrt(src.x * src.x + src.y * src.y + src.z * src.z);
                if (depth > 60) continue;

                PosPoint dst_point;
                dst_point.x = src.x;
                dst_point.y = src.y;
                dst_point.z = src.z;
                dst_point.timestamp = timebase;

                PosPoint point_raw;
                // t_offset wrt. first point
                point_raw.timestamp = timebase;
                // laser id
                point_raw.x = src.line;
                // angle rad
                point_raw.y = timebase;
                // range m
                point_raw.z = depth;

                output.full_features->at(w, h) = dst_point;
                output.raw_data->at(w, h) = point_raw;

            }
        }
        }
private:
    double livox_ring_No_;
    int num_firing_;
};

} // namespace liso