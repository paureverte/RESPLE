#pragma once

// One adapter per LiDAR driver message type. Each adapter knows how to turn its
// native ROS message into pcl::PointXYZINormal: x/y/z, intensity = per-point time
// relative to the message's header.stamp (ms), curvature = reflectivity/intensity
// used only for display coloring. Everything generic (blind-range filtering,
// monotonic-time dedup, point_filter_num decimation) lives in the callers
// (RESPLE::genericLidarCallback, Mapping.cpp's GenericLidarBuff), not here.
//
// startIndex() is 1 only for LivoxCustomMsgAdapter: index 0 seeds its duplicate-
// point check, the only per-type structural filter (tag/line + consecutive-
// duplicate rejection) among the four. get() returns false for a point that
// fails that filter; the other adapters have none and always return true.

#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include "utils/common_utils.h"
#include "livox_ros_driver2/msg/custom_msg.hpp"

struct OusterAdapter {
    using Msg = sensor_msgs::msg::PointCloud2;
    static constexpr const char* kTypeName = "Ouster";
    // Ouster's native intensity range is much wider than Livox's 0-255
    // reflectivity; global-map visualization scales it down to look comparable.
    static constexpr float kMappingCurvatureScale = 0.1f;

    pcl::PointCloud<ouster_ros::Point> cloud;

    OusterAdapter(const Msg& msg, const LidarConfig&, const rclcpp::Time&) {
        pcl::fromROSMsg(msg, cloud);
    }
    size_t size() const { return cloud.size(); }
    size_t startIndex() const { return 0; }
    bool get(size_t i, pcl::PointXYZINormal& pt) const {
        const auto& p = cloud.points[i];
        pt.x = p.x; pt.y = p.y; pt.z = p.z;
        pt.intensity = float(p.t) / 1e6f;   // already relative, ns -> ms
        pt.curvature = p.intensity;
        return true;
    }
};

struct HesaiAdapter {
    using Msg = sensor_msgs::msg::PointCloud2;
    static constexpr const char* kTypeName = "Hesai";
    static constexpr float kMappingCurvatureScale = 1.0f;

    pcl::PointCloud<hesai_ros::Point> cloud;
    rclcpp::Time stamp_begin;

    HesaiAdapter(const Msg& msg, const LidarConfig&, const rclcpp::Time& t0) : stamp_begin(t0) {
        pcl::fromROSMsg(msg, cloud);
    }
    size_t size() const { return cloud.size(); }
    size_t startIndex() const { return 0; }
    bool get(size_t i, pcl::PointXYZINormal& pt) const {
        const auto& p = cloud.points[i];
        pt.x = p.x; pt.y = p.y; pt.z = p.z;
        double sec;
        double nsec = std::modf(p.timestamp, &sec);   // absolute epoch seconds -> relative ms
        rclcpp::Time t(static_cast<int32_t>(sec), static_cast<int32_t>(nsec * 1.0e9), RCL_ROS_TIME);
        pt.intensity = (t - stamp_begin).seconds() * 1.0e3;
        pt.curvature = p.intensity;
        return true;
    }
};

struct Mid360Adapter {
    using Msg = sensor_msgs::msg::PointCloud2;
    static constexpr const char* kTypeName = "Mid360";
    static constexpr float kMappingCurvatureScale = 1.0f;

    pcl::PointCloud<livox_mid360::Point> cloud;
    rclcpp::Time stamp_begin;

    Mid360Adapter(const Msg& msg, const LidarConfig&, const rclcpp::Time& t0) : stamp_begin(t0) {
        pcl::fromROSMsg(msg, cloud);
    }
    size_t size() const { return cloud.size(); }
    size_t startIndex() const { return 0; }
    bool get(size_t i, pcl::PointXYZINormal& pt) const {
        const auto& p = cloud.points[i];
        pt.x = p.x; pt.y = p.y; pt.z = p.z;
        rclcpp::Time t(static_cast<int64_t>(p.timestamp), RCL_ROS_TIME);   // absolute epoch ns -> relative ms
        pt.intensity = (t - stamp_begin).seconds() * 1.0e3;
        pt.curvature = p.intensity;
        return true;
    }
};

struct LivoxCustomMsgAdapter {
    using Msg = livox_ros_driver2::msg::CustomMsg;
    static constexpr const char* kTypeName = "LivoxCustomMsg";
    static constexpr float kMappingCurvatureScale = 1.0f;

    const Msg& msg;
    int scan_line;
    mutable pcl::PointXYZINormal pt_prev;

    LivoxCustomMsgAdapter(const Msg& m, const LidarConfig& lidar, const rclcpp::Time&)
        : msg(m), scan_line(lidar.scan_line) {
        pt_prev.x = msg.points[0].x;
        pt_prev.y = msg.points[0].y;
        pt_prev.z = msg.points[0].z;
    }
    size_t size() const { return msg.point_num; }
    size_t startIndex() const { return 1; }
    bool get(size_t i, pcl::PointXYZINormal& pt) const {
        const auto& p = msg.points[i];
        if (!(p.line < scan_line && ((p.tag & 0x30) == 0x10 || (p.tag & 0x30) == 0x00))) {
            return false;
        }
        pt.x = p.x; pt.y = p.y; pt.z = p.z;
        pt.intensity = float(p.offset_time) / 1e6f;   // already relative, ns -> ms
        pt.curvature = p.reflectivity;
        bool is_duplicate = std::abs(pt.x - pt_prev.x) <= 1e-7 &&
                             std::abs(pt.y - pt_prev.y) <= 1e-7 &&
                             std::abs(pt.z - pt_prev.z) <= 1e-7;
        pt_prev = pt;
        return !is_duplicate;
    }
};
