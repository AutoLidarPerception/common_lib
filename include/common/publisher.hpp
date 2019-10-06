/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */

#ifndef COMMON_INCLUDE_COMMON_PUBLISHER_HPP_
#define COMMON_INCLUDE_COMMON_PUBLISHER_HPP_

#include <pcl/common/common.h>  // pcl::getMinMax3D
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>  // std_msgs, sensor_msgs
#include <std_msgs/ColorRGBA.h>       // std_msgs::ColorRGBA
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>  // visualization_msgs::MarkerArray
#include <vector>
#include <map>
#include <algorithm>

#include "common/msgs/autosense_msgs/PointCloud2Array.h"
#include "common/msgs/autosense_msgs/TrackingFixedTrajectoryArray.h"
#include "common/msgs/autosense_msgs/TrackingObjectArray.h"

#include "common/common.hpp"
#include "common/geometry.hpp"      // common::geometry::calcYaw4DirectionVector
#include "common/transform.hpp"     // common::transform::transformPointCloud
#include "common/types/object.hpp"  // ObjectPtr
#include "common/types/type.h"

namespace autosense {
namespace common {

template <typename PointT>
static void publishCloud(const ros::Publisher &publisher,
                         const std_msgs::Header &header,
                         const typename pcl::PointCloud<PointT> &cloud) {
    if (cloud.size()) {
        sensor_msgs::PointCloud2 msg_cloud;
        pcl::toROSMsg(cloud, msg_cloud);
        msg_cloud.header = header;
        publisher.publish(msg_cloud);
    }
}

/**
 * @brief publish clustering objects' in one point cloud
 * @param publisher
 * @param header
 * @param cloud_clusters
 * @param trans
 */
template <typename PointT>
static void publishClustersCloud(
    const ros::Publisher &publisher,
    const std_msgs::Header &header,
    const std::vector<typename pcl::PointCloud<PointT>::Ptr> &clusters_array,
    const Eigen::Matrix4d &trans = Eigen::Matrix4d::Zero()) {
    if (clusters_array.size() <= 0) {
        ROS_WARN("Publish empty clusters cloud.");
        // publish empty cloud
        sensor_msgs::PointCloud2 msg_cloud;
        pcl::toROSMsg(*(new PointICloud), msg_cloud);
        msg_cloud.header = header;
        publisher.publish(msg_cloud);
        return;
    } else {
        ROS_INFO_STREAM("Publishing " << clusters_array.size()
                                      << " clusters in one cloud.");
    }

    PointICloudPtr cloud(new PointICloud);
    // different clusters with different intensity
    float step_i = 255.0f / clusters_array.size();
    for (size_t cluster_idx = 0u; cluster_idx < clusters_array.size();
         ++cluster_idx) {
        if (clusters_array[cluster_idx]->points.size() <= 0) {
            ROS_WARN_STREAM("An empty cluster #" << cluster_idx << ".");
            continue;
        }
        for (size_t idx = 0u; idx < clusters_array[cluster_idx]->points.size();
             ++idx) {
            PointI point;
            point.x = clusters_array[cluster_idx]->points[idx].x;
            point.y = clusters_array[cluster_idx]->points[idx].y;
            point.z = clusters_array[cluster_idx]->points[idx].z;
            point.intensity = cluster_idx * step_i;
            cloud->points.push_back(point);
        }
    }
    if ((trans.array() != 0.0).any()) {
        common::transform::transformPointCloud<PointI>(trans, cloud);
    }

    if (cloud->size()) {
        sensor_msgs::PointCloud2 msg_cloud;
        pcl::toROSMsg(*cloud, msg_cloud);
        msg_cloud.header = header;
        publisher.publish(msg_cloud);
    }
}

static void publishClustersCloud(
    const ros::Publisher &publisher,
    const std_msgs::Header &header,
    const std::vector<ObjectPtr> &objects_array,
    const Eigen::Matrix4d &trans = Eigen::Matrix4d::Zero()) {
    if (objects_array.size() <= 0) {
        ROS_WARN("Publish empty clusters cloud.");
        // publish empty cloud
        sensor_msgs::PointCloud2 msg_cloud;
        pcl::toROSMsg(*(new PointICloud), msg_cloud);
        msg_cloud.header = header;
        publisher.publish(msg_cloud);
        return;
    } else {
        ROS_INFO_STREAM("Publishing " << objects_array.size()
                                      << " clusters in one cloud.");
    }

    PointICloudPtr cloud(new PointICloud);
    // different clusters with different intensity
    float step_i = 255.0f / objects_array.size();
    for (size_t cluster_idx = 0u; cluster_idx < objects_array.size();
         ++cluster_idx) {
        PointICloudConstPtr cloud_tmp(objects_array[cluster_idx]->cloud);
        if (cloud_tmp->points.size() <= 0) {
            ROS_WARN_STREAM("An empty cluster #" << cluster_idx << ".");
            continue;
        }
        for (size_t idx = 0u; idx < cloud_tmp->points.size(); ++idx) {
            PointI point;
            point.x = cloud_tmp->points[idx].x;
            point.y = cloud_tmp->points[idx].y;
            point.z = cloud_tmp->points[idx].z;
            point.intensity = cluster_idx * step_i;
            cloud->points.push_back(point);
        }
    }
    if ((trans.array() != 0.0).any()) {
        common::transform::transformPointCloud<PointI>(trans, cloud);
    }

    if (cloud->size()) {
        sensor_msgs::PointCloud2 msg_cloud;
        pcl::toROSMsg(*cloud, msg_cloud);
        msg_cloud.header = header;
        publisher.publish(msg_cloud);
    }
}

/**
 * @brief publish self-defined clouds array
 * @tparam CloudT
 * @param publisher
 * @param header
 * @param cloud_segments
 */
template <typename CloudT>
static void publishPointCloudArray(const ros::Publisher &publisher,
                                   const std_msgs::Header &header,
                                   const std::vector<CloudT> &segment_array) {
    if (segment_array.empty()) {
        ROS_WARN("Publish empty result segments.");
        // publish empty cloud array
        autosense_msgs::PointCloud2Array segments_msg;
        segments_msg.header = header;
        publisher.publish(segments_msg);

        return;
    } else {
        ROS_INFO_STREAM("Publishing " << segment_array.size() << " segments.");

        autosense_msgs::PointCloud2Array segments_msg;
        std::vector<sensor_msgs::PointCloud2> clouds;

        for (size_t idx = 0u; idx < segment_array.size(); ++idx) {
            if (segment_array[idx]->points.size() <= 0) {
                ROS_WARN_STREAM("An empty Segment #" << idx << ".");
                continue;
            }
            sensor_msgs::PointCloud2 cloud;
            pcl::toROSMsg(*segment_array[idx], cloud);
            clouds.push_back(cloud);
        }

        if (!clouds.empty()) {
            segments_msg.header = header;
            segments_msg.clouds = clouds;

            publisher.publish(segments_msg);
        }
    }
}

/**
 * @brief publish Objects' 3D OBB and velocity arrow
 * @param publisher
 * @param header
 * @param color
 * @param objects
 * @param trans
 */
static void publishObjectsMarkers(
    const ros::Publisher &publisher,
    const std_msgs::Header &header,
    const std_msgs::ColorRGBA &color,
    const std::vector<ObjectPtr> &objects_array,
    const Eigen::Matrix4d &trans = Eigen::Matrix4d::Zero()) {
    // clear all markers before
    visualization_msgs::MarkerArray empty_markers;
    visualization_msgs::Marker clear_marker;
    clear_marker.header = header;
    clear_marker.ns = "objects";
    clear_marker.id = 0;
    clear_marker.action = clear_marker.DELETEALL;
    clear_marker.lifetime = ros::Duration();
    empty_markers.markers.push_back(clear_marker);
    publisher.publish(empty_markers);

    if (objects_array.size() <= 0) {
        ROS_WARN("Publish empty object marker.");
        return;
    } else {
        ROS_INFO("Publishing %lu objects markers.", objects_array.size());
    }

    visualization_msgs::MarkerArray object_markers;
    for (size_t obj = 0u; obj < objects_array.size(); ++obj) {
        /*
         * @note Apollo's Object Coordinate
         *          |x
         *      C   |   D-----------
         *          |              |
         *  y---------------     length
         *          |              |
         *      B   |   A-----------
         */
        Eigen::Vector3d center = objects_array[obj]->ground_center;
        Eigen::Vector3d dir = objects_array[obj]->direction;
        if ((trans.array() != 0.0).any()) {
            common::transform::transformGroundBox(trans, &center);
            common::transform::transformDirection(trans, &dir);
        }
        // object size
        const double &length = objects_array[obj]->length;
        const double &width = objects_array[obj]->width;
        const double &height = objects_array[obj]->height;
        const double yaw = common::geometry::calcYaw4DirectionVector(dir);
        Eigen::Vector3d ldir(cos(yaw), sin(yaw), 0);
        Eigen::Vector3d odir(-ldir[1], ldir[0], 0);
        Eigen::Vector3d bottom_quad[8];
        double half_l = length / 2;
        double half_w = width / 2;
        double h = height;
        // A(-half_l, -half_w)
        bottom_quad[0] = center + ldir * -half_l + odir * -half_w;
        // B(-half_l, half_w)
        bottom_quad[1] = center + ldir * -half_l + odir * half_w;
        // C(half_l, half_w)
        bottom_quad[2] = center + ldir * half_l + odir * half_w;
        // D(half_l, -half_w)
        bottom_quad[3] = center + ldir * half_l + odir * -half_w;
        // top 4 vertices
        bottom_quad[4] = bottom_quad[0];
        bottom_quad[4](2) += h;
        bottom_quad[5] = bottom_quad[1];
        bottom_quad[5](2) += h;
        bottom_quad[6] = bottom_quad[2];
        bottom_quad[6](2) += h;
        bottom_quad[7] = bottom_quad[3];
        bottom_quad[7](2) += h;

        Eigen::MatrixXf OBB(8, 3);
        OBB << bottom_quad[0](0), bottom_quad[0](1), bottom_quad[0](2),
            bottom_quad[1](0), bottom_quad[1](1), bottom_quad[1](2),
            bottom_quad[2](0), bottom_quad[2](1), bottom_quad[2](2),
            bottom_quad[3](0), bottom_quad[3](1), bottom_quad[3](2),
            bottom_quad[4](0), bottom_quad[4](1), bottom_quad[4](2),
            bottom_quad[5](0), bottom_quad[5](1), bottom_quad[5](2),
            bottom_quad[6](0), bottom_quad[6](1), bottom_quad[6](2),
            bottom_quad[7](0), bottom_quad[7](1), bottom_quad[7](2);

        visualization_msgs::Marker box, dir_arrow;
        box.header = dir_arrow.header = header;
        box.ns = dir_arrow.ns = "objects";
        box.id = obj;
        dir_arrow.id = obj + objects_array.size();
        box.type = visualization_msgs::Marker::LINE_LIST;
        dir_arrow.type = visualization_msgs::Marker::ARROW;
        geometry_msgs::Point p[24];
        // Ground side
        // A->B
        p[0].x = OBB(0, 0);
        p[0].y = OBB(0, 1);
        p[0].z = OBB(0, 2);
        p[1].x = OBB(1, 0);
        p[1].y = OBB(1, 1);
        p[1].z = OBB(1, 2);
        // B->C
        p[2].x = OBB(1, 0);
        p[2].y = OBB(1, 1);
        p[2].z = OBB(1, 2);
        p[3].x = OBB(2, 0);
        p[3].y = OBB(2, 1);
        p[3].z = OBB(2, 2);
        // C->D
        p[4].x = OBB(2, 0);
        p[4].y = OBB(2, 1);
        p[4].z = OBB(2, 2);
        p[5].x = OBB(3, 0);
        p[5].y = OBB(3, 1);
        p[5].z = OBB(3, 2);
        // D->A
        p[6].x = OBB(3, 0);
        p[6].y = OBB(3, 1);
        p[6].z = OBB(3, 2);
        p[7].x = OBB(0, 0);
        p[7].y = OBB(0, 1);
        p[7].z = OBB(0, 2);

        // Top side
        // E->F
        p[8].x = OBB(4, 0);
        p[8].y = OBB(4, 1);
        p[8].z = OBB(4, 2);
        p[9].x = OBB(5, 0);
        p[9].y = OBB(5, 1);
        p[9].z = OBB(5, 2);
        // F->G
        p[10].x = OBB(5, 0);
        p[10].y = OBB(5, 1);
        p[10].z = OBB(5, 2);
        p[11].x = OBB(6, 0);
        p[11].y = OBB(6, 1);
        p[11].z = OBB(6, 2);
        // G->H
        p[12].x = OBB(6, 0);
        p[12].y = OBB(6, 1);
        p[12].z = OBB(6, 2);
        p[13].x = OBB(7, 0);
        p[13].y = OBB(7, 1);
        p[13].z = OBB(7, 2);
        // H->E
        p[14].x = OBB(7, 0);
        p[14].y = OBB(7, 1);
        p[14].z = OBB(7, 2);
        p[15].x = OBB(4, 0);
        p[15].y = OBB(4, 1);
        p[15].z = OBB(4, 2);

        // Around side
        // A->E
        p[16].x = OBB(0, 0);
        p[16].y = OBB(0, 1);
        p[16].z = OBB(0, 2);
        p[17].x = OBB(4, 0);
        p[17].y = OBB(4, 1);
        p[17].z = OBB(4, 2);
        // B->F
        p[18].x = OBB(1, 0);
        p[18].y = OBB(1, 1);
        p[18].z = OBB(1, 2);
        p[19].x = OBB(5, 0);
        p[19].y = OBB(5, 1);
        p[19].z = OBB(5, 2);
        // C->G
        p[20].x = OBB(2, 0);
        p[20].y = OBB(2, 1);
        p[20].z = OBB(2, 2);
        p[21].x = OBB(6, 0);
        p[21].y = OBB(6, 1);
        p[21].z = OBB(6, 2);
        // D->H
        p[22].x = OBB(3, 0);
        p[22].y = OBB(3, 1);
        p[22].z = OBB(3, 2);
        p[23].x = OBB(7, 0);
        p[23].y = OBB(7, 1);
        p[23].z = OBB(7, 2);

        for (size_t pi = 0u; pi < 24; ++pi) {
            box.points.push_back(p[pi]);
        }
        box.scale.x = 0.1;
        box.color = color;
        object_markers.markers.push_back(box);

        // direction
        geometry_msgs::Point start_point, end_point;
        Eigen::Vector3d end = center + dir * (length);
        start_point.x = center[0];
        start_point.y = center[1];
        start_point.z = center[2];
        end_point.x = end[0];
        end_point.y = end[1];
        end_point.z = end[2];
        dir_arrow.points.push_back(start_point);
        dir_arrow.points.push_back(end_point);
        dir_arrow.scale.x = 0.1;
        dir_arrow.scale.y = 0.2;
        dir_arrow.scale.z = length * 0.1;
        dir_arrow.color.a = 1.0;
        dir_arrow.color.r = 1.0;
        dir_arrow.color.g = 0.0;
        dir_arrow.color.b = 0.0;

        object_markers.markers.push_back(dir_arrow);
    }

    publisher.publish(object_markers);
}

static void publishObjectsVelocityArrow(
    const ros::Publisher &publisher,
    const std_msgs::Header &header,
    const std_msgs::ColorRGBA &color,
    const std::vector<ObjectPtr> &objects_array,
    const bool &is_offline_keep_alive = true) {
    // clear all markers before
    visualization_msgs::MarkerArray empty_markers;
    visualization_msgs::Marker clear_marker;
    clear_marker.header = header;
    clear_marker.ns = "tracking_vels";
    clear_marker.id = 0;
    // clear_marker.type = clear_marker.TEXT_VIEW_FACING;
    // clear_marker.action = clear_marker.DELETEALL;
    clear_marker.action = clear_marker.DELETEALL;
    clear_marker.lifetime = ros::Duration();
    empty_markers.markers.push_back(clear_marker);
    publisher.publish(empty_markers);

    if (objects_array.empty()) {
        ROS_WARN("Publish empty object's velocity.");
        return;
    } else {
        ROS_INFO_STREAM("Publishing " << objects_array.size()
                                      << " objects' velocity.");
    }

    if (!objects_array.empty()) {
        visualization_msgs::MarkerArray velocity_markers;
        uint32_t id = 0u;
        for (size_t obj = 0u; obj < objects_array.size(); ++obj) {
            visualization_msgs::Marker vel, vel_text;
            vel.header = vel_text.header = header;
            vel.ns = "tracking_vels";
            vel_text.ns = "tracking_vels";
            vel.id = id++;
            vel_text.id = id++;
            vel.type = visualization_msgs::Marker::ARROW;
            vel_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            // Fill in velocity's arrow
            geometry_msgs::Point start_point, end_point;
            const Eigen::Vector3d &center = objects_array[obj]->ground_center;
            const Eigen::Vector3d &velocity = objects_array[obj]->velocity;
            const double &yaw_rad = objects_array[obj]->yaw_rad;
            const double &length = objects_array[obj]->length;
            const double &height = objects_array[obj]->height;
            Eigen::Vector3d dir(cos(yaw_rad), sin(yaw_rad), 0);
            Eigen::Vector3d start, end;
            if (dir.dot(velocity) < 0) {
                start = center - dir * (length / 2);
            } else {
                start = center + dir * (length / 2);
            }
            end = start + velocity;
            start_point.x = start(0);
            start_point.y = start(1);
            start_point.z = start(2);
            end_point.x = end(0);
            end_point.y = end(1);
            end_point.z = end(2);
            /*ROS_WARN_STREAM("Object #" << obj << ": "
                                       <<"("<<  start_point.x << ", " <<
               start_point.y << ", " << start_point.z <<") --> "
                                       <<"("<<  end_point.x << ", " <<
               end_point.y << ", " << end_point.z <<")");*/
            vel.points.push_back(start_point);
            vel.points.push_back(end_point);
            vel.scale.x = 0.1;
            vel.scale.y = 0.2;
            /// 10% velocity scalar as arrow header
            double vel_scalar =
                sqrt(pow(velocity[0], 2.0) + pow(velocity[1], 2.0));
            vel.scale.z = vel_scalar * 0.1;
            vel.color = color;
            if (!is_offline_keep_alive) {
                vel.lifetime = ros::Duration(0.5);
            }
            velocity_markers.markers.push_back(vel);

            // Fill in velocity's label
            vel_text.pose.position.x = end(0);
            vel_text.pose.position.y = end(1);
            vel_text.pose.position.z = center[2];
            // Eigen::Quaternion q = Eigen::AngleAxisd(yaw_rad,
            // Eigen::Vector3f::UnitZ());
            vel_text.pose.orientation.w = yaw_rad;
            /// @note filter 0.00m/s
            if (vel_scalar > common::EPSILON) {
                std::stringstream vel_str;
                // fixed：表示普通方式输出，不采用科学计数法。
                vel_str << std::fixed << std::setprecision(2)
                        << std::setfill('0') << vel_scalar;
                vel_text.text = vel_str.str() + " m/s";
            }
            vel_text.scale.z = 0.7;
            vel_text.color = color;
            if (!is_offline_keep_alive) {
                vel_text.lifetime = ros::Duration(0.5);
            }
            velocity_markers.markers.push_back(vel_text);
        }

        publisher.publish(velocity_markers);
    }
}

/**
 * @brief publish current tracking objects' trajectories
 * @param publisher
 *  trajectory_pub_, trajectory's marker array publish
 * @param header
 * @param world2velo_trans
 *  project into Velodyne local frame
 * @param trajectories
 *  Id->Trajectory map array, with Trajectory as poses array
 */
static void publishObjectsTrajectory(
    const ros::Publisher &publisher,
    const std_msgs::Header &header,
    const Eigen::Matrix4d &world2velo_trans,
    const std::map<IdType, Trajectory> &trajectories,
    const bool &is_offline_keep_alive = true) {
    // clear all markers before
    visualization_msgs::MarkerArray empty_markers;
    visualization_msgs::Marker clear_marker;
    clear_marker.header = header;
    clear_marker.ns = "tracking_trajectory";
    clear_marker.id = 0;
    clear_marker.action = clear_marker.DELETEALL;
    clear_marker.lifetime = ros::Duration();
    empty_markers.markers.push_back(clear_marker);
    publisher.publish(empty_markers);

    if (trajectories.empty()) {
        ROS_WARN("Publish empty object's trajectory.");
        return;
    } else {
        ROS_INFO_STREAM("Publishing " << trajectories.size()
                                      << " objects' trajectory.");
    }

    visualization_msgs::MarkerArray markers_trajectory;
    if (!trajectories.empty()) {
        auto iter = trajectories.begin();
        for (; iter != trajectories.end(); ++iter) {
            const IdType &tid = iter->first;
            const Trajectory &poses = iter->second;
            if (poses.size() > 1) {
                visualization_msgs::Marker trajectory;
                trajectory.header = header;
                trajectory.ns = "tracking_trajectory";
                trajectory.id = tid;
                trajectory.type = visualization_msgs::Marker::LINE_STRIP;

                geometry_msgs::Point pose;
                for (size_t idx = 0u; idx < poses.size(); ++idx) {
                    /// TODO transform poses from World Coordinate into Local
                    /// Velodyne
                    Eigen::Vector4d pose_velo =
                        world2velo_trans * Eigen::Vector4d(poses[idx][0],
                                                           poses[idx][1],
                                                           poses[idx][2], 1);
                    pose.x = pose_velo[0];
                    pose.y = pose_velo[1];
                    pose.z = pose_velo[2];
                    trajectory.points.push_back(pose);
                }

                trajectory.scale.x = 0.15;
                trajectory.color.a = 1.0;
                trajectory.color.r =
                    std::max(0.3, static_cast<double>(tid % 3) / 3.0);
                trajectory.color.g =
                    std::max(0.3, static_cast<double>(tid % 6) / 6.0);
                trajectory.color.b =
                    std::max(0.3, static_cast<double>(tid % 9) / 9.0);

                if (!is_offline_keep_alive) {
                    trajectory.lifetime = ros::Duration(0.5);
                }
                markers_trajectory.markers.push_back(trajectory);
            }
        }
    }

    publisher.publish(markers_trajectory);
}

/**
 * @brief publish Clusters' Min-Max Size box
 * @param publisher
 * @param header
 * @param color
 * @param objects
 * @param trans
 */
static void publishMinMaxMarkers(
    const ros::Publisher &publisher,
    const std_msgs::Header &header,
    const std_msgs::ColorRGBA &color,
    const std::vector<PointICloudPtr> &clusters_array,
    const Eigen::Matrix4d &trans = Eigen::Matrix4d::Zero()) {
    // clear all markers before
    visualization_msgs::MarkerArray empty_markers;
    visualization_msgs::Marker clear_marker;
    clear_marker.header = header;
    clear_marker.ns = "clusters";
    clear_marker.id = 0;
    clear_marker.action = clear_marker.DELETEALL;
    clear_marker.lifetime = ros::Duration();
    empty_markers.markers.push_back(clear_marker);
    publisher.publish(empty_markers);

    if (clusters_array.size() <= 0) {
        ROS_WARN("Publish empty cluster marker.");
        return;
    } else {
        ROS_INFO("Publishing %lu clusters markers.", clusters_array.size());
    }

    visualization_msgs::MarkerArray cluster_markers;
    for (size_t seg = 0u; seg < clusters_array.size(); ++seg) {
        PointI pt_min, pt_max;
        pcl::getMinMax3D(*clusters_array[seg], pt_min, pt_max);
        // transform into local coordinate
        if ((trans.array() != 0.0).any()) {
            common::transform::transformPoint<PointI>(trans, &pt_min);
            common::transform::transformPoint<PointI>(trans, &pt_max);
        }

        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "clusters";
        marker.id = seg;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        geometry_msgs::Point p[24];
        p[0].x = pt_max.x;
        p[0].y = pt_max.y;
        p[0].z = pt_max.z;
        p[1].x = pt_min.x;
        p[1].y = pt_max.y;
        p[1].z = pt_max.z;
        p[2].x = pt_max.x;
        p[2].y = pt_max.y;
        p[2].z = pt_max.z;
        p[3].x = pt_max.x;
        p[3].y = pt_min.y;
        p[3].z = pt_max.z;
        p[4].x = pt_max.x;
        p[4].y = pt_max.y;
        p[4].z = pt_max.z;
        p[5].x = pt_max.x;
        p[5].y = pt_max.y;
        p[5].z = pt_min.z;
        p[6].x = pt_min.x;
        p[6].y = pt_min.y;
        p[6].z = pt_min.z;
        p[7].x = pt_max.x;
        p[7].y = pt_min.y;
        p[7].z = pt_min.z;
        p[8].x = pt_min.x;
        p[8].y = pt_min.y;
        p[8].z = pt_min.z;
        p[9].x = pt_min.x;
        p[9].y = pt_max.y;
        p[9].z = pt_min.z;
        p[10].x = pt_min.x;
        p[10].y = pt_min.y;
        p[10].z = pt_min.z;
        p[11].x = pt_min.x;
        p[11].y = pt_min.y;
        p[11].z = pt_max.z;
        p[12].x = pt_min.x;
        p[12].y = pt_max.y;
        p[12].z = pt_max.z;
        p[13].x = pt_min.x;
        p[13].y = pt_max.y;
        p[13].z = pt_min.z;
        p[14].x = pt_min.x;
        p[14].y = pt_max.y;
        p[14].z = pt_max.z;
        p[15].x = pt_min.x;
        p[15].y = pt_min.y;
        p[15].z = pt_max.z;
        p[16].x = pt_max.x;
        p[16].y = pt_min.y;
        p[16].z = pt_max.z;
        p[17].x = pt_max.x;
        p[17].y = pt_min.y;
        p[17].z = pt_min.z;
        p[18].x = pt_max.x;
        p[18].y = pt_min.y;
        p[18].z = pt_max.z;
        p[19].x = pt_min.x;
        p[19].y = pt_min.y;
        p[19].z = pt_max.z;
        p[20].x = pt_max.x;
        p[20].y = pt_max.y;
        p[20].z = pt_min.z;
        p[21].x = pt_min.x;
        p[21].y = pt_max.y;
        p[21].z = pt_min.z;
        p[22].x = pt_max.x;
        p[22].y = pt_max.y;
        p[22].z = pt_min.z;
        p[23].x = pt_max.x;
        p[23].y = pt_min.y;
        p[23].z = pt_min.z;
        for (int i = 0; i < 24; i++) {
            marker.points.push_back(p[i]);
        }

        marker.scale.x = 0.1;
        marker.color = color;

        cluster_markers.markers.push_back(marker);
    }
    publisher.publish(cluster_markers);
}

static void publishTrackingObjects(
    const ros::Publisher &publisher,
    const std_msgs::Header &header,
    const std::vector<ObjectPtr> &tracking_objects) {
    if (tracking_objects.empty()) {
        ROS_WARN("Publish empty object.");
        return;
    } else {
        ROS_INFO_STREAM("Publishing " << tracking_objects.size()
                                      << " objects.");
    }

    if (!tracking_objects.empty()) {
        autosense_msgs::TrackingObjectArray msg_tracking_objects;
        msg_tracking_objects.header = header;
        for (size_t i = 0u; i < tracking_objects.size(); ++i) {
            ObjectConstPtr obj = tracking_objects[i];
            // tracker id
            msg_tracking_objects.ids.push_back(obj->tracker_id);
            // position
            geometry_msgs::Point position;
            position.x = obj->ground_center(0);
            position.y = obj->ground_center(1);
            position.z = obj->ground_center(2);
            msg_tracking_objects.positions.push_back(position);
            // direction
            geometry_msgs::Point direction;
            direction.x = obj->direction(0);
            direction.y = obj->direction(1);
            direction.z = obj->direction(2);
            msg_tracking_objects.directions.push_back(direction);
            // size
            geometry_msgs::Point size;
            size.x = obj->length;
            size.y = obj->width;
            size.z = obj->height;
            msg_tracking_objects.sizes.push_back(size);
            // segment
            sensor_msgs::PointCloud2 msg_segment;
            pcl::toROSMsg(*obj->cloud, msg_segment);
            msg_tracking_objects.segments.push_back(msg_segment);
            // velocity
            geometry_msgs::Point velocity;
            velocity.x = obj->velocity(0);
            velocity.y = obj->velocity(1);
            velocity.z = obj->velocity(2);
            msg_tracking_objects.velocities.push_back(velocity);
        }
        publisher.publish(msg_tracking_objects);
    }
}

static void publishTrackingFixedTrajectories(
    const ros::Publisher &publisher,
    const std_msgs::Header &header,
    const std::vector<FixedTrajectory> &trajectories) {
    if (trajectories.empty()) {
        ROS_WARN("Publish empty fixed trajectory.");
        return;
    } else {
        ROS_INFO_STREAM("Publishing " << trajectories.size()
                                      << " fixed trajectories.");
    }

    if (!trajectories.empty()) {
        autosense_msgs::TrackingFixedTrajectoryArrayPtr msg(
            new autosense_msgs::TrackingFixedTrajectoryArray);
        msg->header = header;
        for (size_t i = 0u; i < trajectories.size(); ++i) {
            // tracker id
            msg->ids.push_back(std::get<0>(trajectories[i]));
            // alive period
            msg->periods.push_back(std::get<1>(trajectories[i]));
        }
        publisher.publish(msg);
    }
}

}  // namespace common
}  // namespace autosense

#endif  // COMMON_INCLUDE_COMMON_PUBLISHER_HPP_
