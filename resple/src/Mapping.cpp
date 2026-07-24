#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <thread>
#include <iostream>
#include <queue>
#include <string>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/int64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <rclcpp/service.hpp>
#include <std_srvs/srv/empty.hpp>
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "estimate_msgs/msg/calib.hpp"
#include "estimate_msgs/msg/estimate.hpp"
#include "SplineState.h"
#include "utils/lidar_adapters.h"

template<typename PointType>
class MappingBase
{
  public:

    std::mutex mtx;    
    LidarConfig lidar;
    MappingBase(rclcpp::Node::SharedPtr &nh, const LidarConfig& lidar_config) : lidar(lidar_config)
    {
        odom_id = CommonUtils::readParam<std::string>(nh, "frames.world", std::string("world"));
        pub_global_map = nh->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", 2);
        float ds_map_voxel = CommonUtils::readParam<float>(nh, "mapping.ds_map_voxel", 0.2);
        ds_filter_each_scan.setLeafSize(ds_map_voxel, ds_map_voxel, ds_map_voxel);
        pc_last.reset(new typename pcl::PointCloud<PointType>());
        pc_last_ds.reset(new typename pcl::PointCloud<PointType>());
        pc.reset(new typename pcl::PointCloud<PointType>());   
    }

    void processScan(SplineState* spl, const int64_t spl_window_st_ns)
    {
        int64_t t_end_ns = 0;
        rclcpp::Rate rate(20);
        while (!pc_L_buff.empty()) {
            t_end_ns = pc_L_buff.front().header.stamp + int64_t (pc_L_buff.front().points.back().intensity * float(1e6));
            mtx.lock();
            if (t_end_ns < spl->minTimeNs()) {
                pc_L_buff.pop_front();
                mtx.unlock(); 
            } else if (t_end_ns <= spl->maxTimeNs()) {
                transformCloud(pc_L_buff.front(), spl, pc);
                pc_L_buff.pop_front();
                mtx.unlock();                
                publishMap(pc, pub_global_map);
            } else {
                mtx.unlock(); 
                rate.sleep();
                break;
            }
        }
    }

    void publishMap(const typename pcl::PointCloud<PointType>::Ptr& pcs,
                         const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) const
    {
        sensor_msgs::msg::PointCloud2 msgs;
        pcl::toROSMsg(*pcs, msgs);
        msgs.header.frame_id = odom_id;
        publisher->publish(msgs);
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_map;

    PointType transformPoint(int64_t time_ns, const SplineState* spl, const PointType& pt_in) const
    {     
        Eigen::Quaterniond q_itp;
        Eigen::Vector3d t_itp;
        spl->itpQuaternion(time_ns, &q_itp);
        t_itp = spl->itpPosition(time_ns);
        Eigen::Vector3d p_body(pt_in.x, pt_in.y, pt_in.z);
        Eigen::Vector3d p_imu(lidar.q_bl * p_body + lidar.t_bl);
        Eigen::Vector3d p_global(q_itp * p_imu + t_itp);
        PointType point_world;
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = pt_in.intensity;
        point_world.curvature = pt_in.curvature;
        return point_world;
    }    

    void transformCloud(const typename pcl::PointCloud<PointType>& pc_in, SplineState* spl,
                       typename pcl::PointCloud<PointType>::Ptr pc_out) const
    {
        int64_t time_begin = rclcpp::Time(pc_in.header.stamp).nanoseconds();
        pc->clear();
        pc_out->points.resize(pc_in.size());
        for (size_t i = 0; i < pc_in.size(); i++) {
            const PointType& pt = pc_in.points[i];
            int64_t t_ns = int64_t(pt.intensity * float(1e6)) + time_begin;
            if (t_ns >= spl->minTimeNs() && t_ns <= spl->maxTimeNs()) {
                pc_out->points[i] = transformPoint(t_ns, spl, pt);
            }
        }
    }    

  protected:
    Eigen::aligned_deque<typename pcl::PointCloud<PointType>> pc_L_buff;
    typename pcl::PointCloud<PointType>::Ptr pc_last;
    typename pcl::PointCloud<PointType>::Ptr pc_last_ds;
    pcl::VoxelGrid<pcl::PointXYZINormal> ds_filter_each_scan;
    const std::string frame_id = "body";
    std::string odom_id = "world";
    typename pcl::PointCloud<PointType>::Ptr pc;

};


// Shared by all 4 LiDAR types (see utils/lidar_adapters.h): Adapter absorbs the
// per-type message layout/timestamp convention/structural filter and the
// display-only curvature scale; everything else (downsampling, buffering into
// pc_L_buff) is generic. Unlike RESPLE's ingestion path there is no
// point_filter_num decimation or blind-range filtering here — every
// structurally-valid point is kept and later voxel-downsampled instead.
template<typename Adapter>
class GenericLidarBuff : public MappingBase<pcl::PointXYZINormal>
{
  public:
    GenericLidarBuff(rclcpp::Node::SharedPtr &nh, const LidarConfig& lidar_config) : MappingBase<pcl::PointXYZINormal>(nh, lidar_config)
    {
        pc_subscription = nh->create_subscription<typename Adapter::Msg>(
            this->lidar.topic, 100, std::bind(&GenericLidarBuff::callback, this, std::placeholders::_1));
        double lidar_time_offset = CommonUtils::readParam<double>(nh, "lidar.time_offset", 0.0);
        time_offset = 1e9*lidar_time_offset;
    }

    void callback(const typename Adapter::Msg::SharedPtr msg_in)
    {
        this->pc_last->clear();
        rclcpp::Time stamp_begin(msg_in->header.stamp);
        Adapter adapter(*msg_in, this->lidar, stamp_begin);
        size_t plsize = adapter.size();
        if (plsize == 0) return;
        this->pc_last->reserve(plsize);
        for (size_t i = adapter.startIndex(); i < plsize; i++) {
            pcl::PointXYZINormal pt;
            if (!adapter.get(i, pt)) continue;
            if (pt.intensity < 0) continue;
            pt.curvature *= Adapter::kMappingCurvatureScale;
            this->pc_last->points.push_back(pt);
        }
        int64_t stamp_ns = stamp_begin.nanoseconds() - time_offset;
        this->pc_last->header.frame_id = this->frame_id;
        this->pc_last->header.stamp = stamp_ns;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*this->pc_last, *this->pc_last, indices);
        if (this->pc_last->points.empty()) return;
        ds_filter_each_scan.setInputCloud(pc_last);
        this->pc_last_ds->clear();
        ds_filter_each_scan.filter(*this->pc_last_ds);
        pc_last_ds->header.frame_id = this->frame_id;
        pc_last_ds->header.stamp = stamp_ns;
        mtx.lock();
        this->pc_L_buff.push_back(*pc_last_ds);
        mtx.unlock();
    }

  private:
    typename rclcpp::Subscription<typename Adapter::Msg>::SharedPtr pc_subscription;
    int64_t time_offset = 0;
};

using OusterBuff = GenericLidarBuff<OusterAdapter>;
using HesaiBuff = GenericLidarBuff<HesaiAdapter>;
using Mid360Buff = GenericLidarBuff<Mid360Adapter>;
using LivoxCustomMsgBuff = GenericLidarBuff<LivoxCustomMsgAdapter>;

class Mapping
{

public:

Mapping(rclcpp::Node::SharedPtr &nh, std::vector<MappingBase<pcl::PointXYZINormal>*>& mappings)
    {
        odom_id = CommonUtils::readParam<std::string>(nh, "frames.world", std::string("world"));
        sub_start = nh->create_subscription<std_msgs::msg::Int64>("/start_time", 100, std::bind(&Mapping::startCallBack, this, std::placeholders::_1));
        spl_window_st_ns = 0;
        sub_est = nh->create_subscription<estimate_msgs::msg::Estimate>("/est_window", 10000, std::bind(&Mapping::getEstCallback, this, std::placeholders::_1));
        pub_path = nh->create_publisher<nav_msgs::msg::Path>("traj_path", 100);
        pub_knots = nh->create_publisher<sensor_msgs::msg::PointCloud>("active_control_points",20);
        opt_old_path.header.frame_id = odom_id;
        vis_maps = mappings;
        pub_odom = nh->create_publisher<nav_msgs::msg::Odometry>("odometry", 500);
        br = std::make_shared<tf2_ros::TransformBroadcaster>(nh);
        tf_buffer = std::make_unique<tf2_ros::Buffer>(nh->get_clock());
        tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

        // target_link: an extra world->TF frame you can point anywhere, e.g. your robot's real
        // base_link. frame_id names the published TF child frame; q_target/t_target is the
        // IMU(body)->target extrinsic, same calibration convention as lidar's q_lb/t_lb.
        // Defaults to identity and "base_link", i.e. base_link == body.
        //
        // frames.odom: if a live <frames.odom> -> <target_link.frame_id> transform is already
        // being published by something else (e.g. your robot's wheel-odometry stack), RESPLE
        // publishes world -> <frames.odom> instead of world -> <frame_id> directly, so it
        // corrects that existing chain (REP-105 map->odom->base_link style) rather than fighting
        // it for the same child frame name. Falls back to publishing world -> <frame_id> directly
        // when no such transform is available (e.g. no odometry source at all).
        target_link_frame_id = CommonUtils::readParam<std::string>(nh, "target_link.frame_id", std::string("base_link"));
        odom_frame_id = CommonUtils::readParam<std::string>(nh, "frames.odom", std::string("odom"));
        std::vector<double> q_blbase_v = CommonUtils::readParam<std::vector<double>>(
            nh, "target_link.q_target", std::vector<double>{1.0, 0.0, 0.0, 0.0});
        Eigen::Quaterniond q_blbase(q_blbase_v.at(0), q_blbase_v.at(1), q_blbase_v.at(2), q_blbase_v.at(3));
        std::vector<double> t_blbase_v = CommonUtils::readParam<std::vector<double>>(
            nh, "target_link.t_target", std::vector<double>{0.0, 0.0, 0.0});
        Eigen::Vector3d t_blbase(t_blbase_v.at(0), t_blbase_v.at(1), t_blbase_v.at(2));
        q_body_target = q_blbase.inverse();
        t_body_target = q_blbase.inverse() * (- t_blbase);
    }

    void lock_mappings() {
        for (auto vis_map : vis_maps) {
            vis_map->mtx.lock();
        }
    }

    void unlock_mappings() {
        for (auto vis_map : vis_maps) {
            vis_map->mtx.unlock();
        }
    }    

    void process() {
        rclcpp::Rate rate(20);
        int64_t num_knot = 0;
        while (true) {
            if (if_init_succeed && spline_global.numKnots() > num_knot) {
                lock_mappings();
                publishPath();
                displayControlPoints();
                pubOdom();
                num_knot = spline_global.numKnots();
                unlock_mappings();
            }
            if (!if_init_succeed) {
                rate.sleep();
                continue;
            }
            for (const auto vis_map : vis_maps) {
                vis_map->processScan(&spline_global, spl_window_st_ns);  
            }                     
        }
    }

private:
    std::string node_name = "Mapping";
    int64_t spl_window_st_ns;
    SplineState spline_global;
    rclcpp::Subscription<estimate_msgs::msg::Estimate>::SharedPtr sub_est;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_start;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    nav_msgs::msg::Path opt_old_path;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_knots;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
    std::vector<MappingBase<pcl::PointXYZINormal>*> vis_maps;
    const std::string frame_id = "body";
    std::string odom_id = "world";
    std::string target_link_frame_id = "base_link";
    std::string odom_frame_id = "odom";
    Eigen::Quaterniond q_body_target = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t_body_target = Eigen::Vector3d::Zero();
    std::shared_ptr<tf2_ros::TransformBroadcaster> br;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;
    bool if_init_succeed = false;
    std::mutex m_spline;

    void displayControlPoints()
    {
        sensor_msgs::msg::PointCloud points_msg;
        points_msg.header.frame_id = odom_id;
        points_msg.header.stamp = rclcpp::Time(spline_global.minTimeNs());
        for (int64_t i = spline_global.numKnots() - 4; i < spline_global.numKnots(); i++) {
            points_msg.points.push_back(CommonUtils::getPointMsg(spline_global.getKnotPos(i)));
        }
        pub_knots->publish(points_msg);
    }

    void getEstCallback(const estimate_msgs::msg::Estimate::SharedPtr est_msg)
    {
        if (!if_init_succeed) {
            return;
        }
        estimate_msgs::msg::Spline spline_msg = est_msg->spline;
        SplineState spline_w;
        
        spline_w.init(spline_msg.dt, 0, spline_msg.start_t, spline_msg.start_idx);
        for(const auto& knot : spline_msg.knots) {
            Eigen::Vector3d pos(knot.position.x, knot.position.y, knot.position.z);
            Eigen::Vector3d quat_del(knot.orientation_del.x, knot.orientation_del.y, knot.orientation_del.z);
            spline_w.addOneStateKnot(pos, quat_del);
        }
        Eigen::Quaterniond q_idle0 = Eigen::Quaterniond(spline_msg.start_q.w, spline_msg.start_q.x, spline_msg.start_q.y, spline_msg.start_q.z);
        for (int i = 0; i < 3; i++) {
            estimate_msgs::msg::Knot idle = spline_msg.idles[i];
            Eigen::Vector3d t_idle(idle.position.x, idle.position.y, idle.position.z);
            Eigen::Vector3d quat_idle(idle.orientation_del.x, idle.orientation_del.y, idle.orientation_del.z);
            spline_w.setIdles(i, t_idle, quat_idle, q_idle0);
        }
        lock_mappings();
        spl_window_st_ns = spline_msg.start_t - spline_msg.dt; 
        spline_global.setTimeIntervalNs(spline_msg.dt);
        spline_global.updateKnots(&spline_w);
        unlock_mappings();
    }

    void pubOdom()
    {
        if (opt_old_path.poses.empty()) {
            return;
        }
        nav_msgs::msg::Odometry odom_msg;
        geometry_msgs::msg::PoseStamped odom_pose = opt_old_path.poses.back();
        odom_msg.header.stamp = rclcpp::Time(odom_pose.header.stamp);
        odom_msg.header.frame_id = odom_id;
        odom_msg.child_frame_id = frame_id;
        odom_msg.pose.pose = odom_pose.pose;
        pub_odom->publish(odom_msg);      
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = odom_msg.header.stamp;
        transformStamped.header.frame_id = odom_id;
        transformStamped.child_frame_id = frame_id;
        transformStamped.transform.translation.x = odom_pose.pose.position.x;
        transformStamped.transform.translation.y = odom_pose.pose.position.y;
        transformStamped.transform.translation.z = odom_pose.pose.position.z;
        transformStamped.transform.rotation = odom_pose.pose.orientation;
        br->sendTransform(transformStamped);
        transformStamped.header.stamp = odom_msg.header.stamp;
        transformStamped.header.frame_id = odom_id;
        transformStamped.child_frame_id = "imu_frame";
        transformStamped.transform.translation.x = odom_pose.pose.position.x;
        transformStamped.transform.translation.y = odom_pose.pose.position.y;
        transformStamped.transform.translation.z = odom_pose.pose.position.z;
        transformStamped.transform.rotation.w = odom_pose.pose.orientation.w;
        transformStamped.transform.rotation.x = odom_pose.pose.orientation.x;
        transformStamped.transform.rotation.y = odom_pose.pose.orientation.y;
        transformStamped.transform.rotation.z = odom_pose.pose.orientation.z;
        br->sendTransform(transformStamped);

        Eigen::Quaterniond q_wi(odom_pose.pose.orientation.w, odom_pose.pose.orientation.x, odom_pose.pose.orientation.y, odom_pose.pose.orientation.z);
        Eigen::Vector3d t_wi(odom_pose.pose.position.x, odom_pose.pose.position.y, odom_pose.pose.position.z);
        Eigen::Quaterniond q_w_target = q_wi * q_body_target;
        Eigen::Vector3d t_w_target = q_wi * t_body_target + t_wi;

        // If <odom_frame_id> -> <target_link_frame_id> is already being published live by
        // something else (e.g. wheel odometry), publish world -> odom_frame_id instead, so we
        // correct that existing chain (REP-105 style) instead of fighting it for the same child
        // frame. Otherwise fall back to publishing world -> target_link_frame_id directly.
        std::string tf_child_frame = target_link_frame_id;
        Eigen::Quaterniond q_w_pub = q_w_target;
        Eigen::Vector3d t_w_pub = t_w_target;
        if (tf_buffer->canTransform(odom_frame_id, target_link_frame_id, tf2::TimePointZero)) {
            try {
                geometry_msgs::msg::TransformStamped odom_to_target =
                    tf_buffer->lookupTransform(odom_frame_id, target_link_frame_id, tf2::TimePointZero);
                Eigen::Quaterniond q_odom_target(odom_to_target.transform.rotation.w, odom_to_target.transform.rotation.x,
                    odom_to_target.transform.rotation.y, odom_to_target.transform.rotation.z);
                Eigen::Vector3d t_odom_target(odom_to_target.transform.translation.x, odom_to_target.transform.translation.y,
                    odom_to_target.transform.translation.z);
                Eigen::Quaterniond q_world_odom = q_w_target * q_odom_target.inverse();
                Eigen::Vector3d t_world_odom = t_w_target - (q_world_odom * t_odom_target);
                tf_child_frame = odom_frame_id;
                q_w_pub = q_world_odom;
                t_w_pub = t_world_odom;
            } catch (const tf2::TransformException& ex) {
                // Race between canTransform and lookupTransform (e.g. buffer expired the data);
                // fall back to publishing world -> target_link_frame_id directly this cycle.
            }
        }
        transformStamped.header.stamp = odom_msg.header.stamp;
        transformStamped.header.frame_id = odom_id;
        transformStamped.child_frame_id = tf_child_frame;
        transformStamped.transform.translation.x = t_w_pub.x();
        transformStamped.transform.translation.y = t_w_pub.y();
        transformStamped.transform.translation.z = t_w_pub.z();
        transformStamped.transform.rotation.w = q_w_pub.w();
        transformStamped.transform.rotation.x = q_w_pub.x();
        transformStamped.transform.rotation.y = q_w_pub.y();
        transformStamped.transform.rotation.z = q_w_pub.z();
        br->sendTransform(transformStamped);

        int id_lidar = 0;
        for (const auto vis_map : vis_maps) {
            Eigen::Quaterniond q_wl = q_wi * vis_map->lidar.q_bl;
            Eigen::Vector3d t_wl = q_wi * vis_map->lidar.t_bl + t_wi;
            std::string lidar_str = "lidar" + std::to_string(id_lidar) + "_frame";
            transformStamped.header.stamp = odom_msg.header.stamp;
            transformStamped.header.frame_id = odom_id;
            transformStamped.child_frame_id = lidar_str;
            transformStamped.transform.translation.x = t_wl.x();
            transformStamped.transform.translation.y = t_wl.y();
            transformStamped.transform.translation.z = t_wl.z();
            transformStamped.transform.rotation.w = q_wl.w();
            transformStamped.transform.rotation.x = q_wl.x();
            transformStamped.transform.rotation.y = q_wl.y();
            transformStamped.transform.rotation.z = q_wl.z();
            br->sendTransform(transformStamped);
            id_lidar++;
        }                 
    }

    void startCallBack(const std_msgs::msg::Int64::SharedPtr start_time_msg)
    {
        int64_t bag_start_time = start_time_msg->data;
        spline_global.init(0, 0, bag_start_time, 0);
        if_init_succeed = true;
    }

    void publishPath() {
        if (!if_init_succeed || spline_global.numKnots() <= 4) {
            return;
        }
        static int64_t t_ns = spline_global.minTimeNs();
        while (t_ns < std::min(spl_window_st_ns, spline_global.maxTimeNs())) {
            Eigen::Quaterniond orient_interp;
            Eigen::Vector3d t_interp = spline_global.itpPosition(t_ns);
            spline_global.itpQuaternion(t_ns, &orient_interp);
            opt_old_path.poses.push_back(CommonUtils::pose2msg(t_ns, t_interp, orient_interp));
            t_ns += 1e8;
        }
        pub_path->publish(opt_old_path);
    }         

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("Mapping");
    std::vector<LidarConfig> lidars;
    auto lidar_names = nh->declare_parameter<std::vector<std::string>>("lidar.names", std::vector<std::string>());
    assert(nh->get_parameter({"lidar.names"}, lidar_names));
    if (lidar_names.empty()) {
        lidars.emplace_back(nh, "lidar.");
    } else {
        for (const auto& lidar_name : lidar_names) {
            lidars.emplace_back(nh, "lidar." + lidar_name + ".");
        }
    }
    std::vector<MappingBase<pcl::PointXYZINormal>*> buffs;
    for (const auto& lidar : lidars) {
        if (!lidar.type.compare("Ouster")) {
            buffs.push_back(new OusterBuff(nh, lidar));
        } else if (!lidar.type.compare("LivoxCustomMsg")) {
            buffs.push_back(new LivoxCustomMsgBuff(nh, lidar));
        } else if (!lidar.type.compare("Hesai")) {
            buffs.push_back(new HesaiBuff(nh, lidar));
        } else if (!lidar.type.compare("Mid360")) {
            buffs.push_back(new Mid360Buff(nh, lidar));
        } else {
            exit(1);
        }
    }
    Mapping mapping(nh, buffs);
    std::thread mappingThread{&Mapping::process, &mapping};
    rclcpp::spin(nh);
    mappingThread.join();
    rclcpp::shutdown();
}