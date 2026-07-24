#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <queue>
#include <thread>
#include <mutex>
#include <boost/make_shared.hpp>
#include <rclcpp/service.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "estimate_msgs/msg/calib.hpp"
#include "estimate_msgs/msg/spline.hpp"
#include "estimate_msgs/msg/estimate.hpp"
#include "estimate_msgs/srv/save_map.hpp"
#include "Estimator.h"
#include "utils/lidar_adapters.h"
#include "Relocalization.h"

KD_TREE<pcl::PointXYZINormal> ikdtree;

class RESPLE
{

public:
    RESPLE(rclcpp::Node::SharedPtr& nh)
    {
        readParameters(nh);
        if (reloc_cfg.enable) {
            if (!Relocalization::loadPriorMap(reloc_cfg.map_path, prior_map_cloud)) {
                RCLCPP_FATAL_STREAM(nh->get_logger(), "Relocalization: failed to load prior map from " << reloc_cfg.map_path);
                exit(1);
            }
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr prior_map_ds(new pcl::PointCloud<pcl::PointXYZINormal>());
            pcl::VoxelGrid<pcl::PointXYZINormal> ds_filter_prior_map;
            ds_filter_prior_map.setLeafSize(ds_lm_voxel, ds_lm_voxel, ds_lm_voxel);
            ds_filter_prior_map.setInputCloud(prior_map_cloud);
            ds_filter_prior_map.filter(*prior_map_ds);
            prior_map_cloud = prior_map_ds;
            // Transient local: published once here, but RViz (or any other late
            // subscriber, e.g. connecting after launch) still receives it.
            pub_prior_map = nh->create_publisher<sensor_msgs::msg::PointCloud2>(
                "prior_map", rclcpp::QoS(1).transient_local());
            sensor_msgs::msg::PointCloud2 prior_map_msg;
            pcl::toROSMsg(*prior_map_cloud, prior_map_msg);
            prior_map_msg.header.frame_id = odom_id;
            pub_prior_map->publish(prior_map_msg);
            if (reloc_cfg.initial_guess) {
                std::lock_guard<std::mutex> lock(m_pose_guess);
                pose_guess_t = reloc_cfg.t0;
                pose_guess_q = reloc_cfg.q0;
                if_have_pose_guess = true;
            } else {
                sub_initial_pose = nh->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                    "/initialpose", 10, std::bind(&RESPLE::getInitialPoseCallback, this, std::placeholders::_1));
            }
        }
        if (!if_lidar_only) {
            std::string imu_type = CommonUtils::readParam<std::string>(nh, "imu.topic");
            sub_imu = nh->create_subscription<sensor_msgs::msg::Imu>(imu_type, 2000000, std::bind(&RESPLE::getImuCallback, this, std::placeholders::_1));
        }
        pub_est = nh->create_publisher<estimate_msgs::msg::Estimate>("est_window", 50);
        pub_start_time = nh->create_publisher<std_msgs::msg::Int64>("start_time", 50);
        pub_cur_scan = nh->create_publisher<sensor_msgs::msg::PointCloud2>("current_scan", 2);
        br = std::make_shared<tf2_ros::TransformBroadcaster>(nh);
        auto lidar_names = nh->declare_parameter<std::vector<std::string>>("lidar.names", std::vector<std::string>());
        assert(nh->get_parameter({"lidar.names"}, lidar_names));
        if (lidar_names.empty()) {
            LidarConfig lidar(nh, "lidar.");
            lidars.emplace(lidar.type, lidar);
            lidars_data.emplace(std::piecewise_construct, std::make_tuple(lidar.type), std::make_tuple());
        } else {
            for (const auto& lidar_name : lidar_names) {
                LidarConfig lidar(nh, "lidar." + lidar_name + ".");
                lidars.emplace(lidar.type, lidar);
                lidars_data.emplace(std::piecewise_construct, std::make_tuple(lidar.type), std::make_tuple());
            }
        }
        if (wheel_cfg.enable) {
            if (!wheel_cfg.topic_type.compare("nav_msgs/msg/Odometry")) {
                sub_wheel_odom = nh->create_subscription<nav_msgs::msg::Odometry>(
                        wheel_cfg.topic, 2000, std::bind(&RESPLE::getWheelOdomCallback, this, std::placeholders::_1));
            } else if (!wheel_cfg.topic_type.compare("geometry_msgs/msg/TwistStamped")) {
                sub_wheel_twist = nh->create_subscription<geometry_msgs::msg::TwistStamped>(
                        wheel_cfg.topic, 2000, std::bind(&RESPLE::getWheelTwistCallback, this, std::placeholders::_1));
            } else {
                RCLCPP_FATAL_STREAM(nh->get_logger(), "Unknown wheel_odometry.topic_type: " << wheel_cfg.topic_type);
                exit(1);
            }
        }
        for (const auto& [lidar_name, lidar] : lidars) {
            if (!lidar.type.compare("Ouster")) {
                sub_ouster = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
                        lidar.topic, 200000, std::bind(&RESPLE::genericLidarCallback<OusterAdapter>, this, std::placeholders::_1));
            } else if (!lidar.type.compare("LivoxCustomMsg")) {
                sub_livox2 = nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                        lidar.topic, 200000, std::bind(&RESPLE::genericLidarCallback<LivoxCustomMsgAdapter>, this, std::placeholders::_1));
            } else if (!lidar.type.compare("Hesai")) {
                sub_hesai = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
                        lidar.topic, 200000, std::bind(&RESPLE::genericLidarCallback<HesaiAdapter>, this, std::placeholders::_1));
            } else if (!lidar.type.compare("Mid360")) {
                sub_livox_mid360 = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
                        lidar.topic, 200000, std::bind(&RESPLE::genericLidarCallback<Mid360Adapter>, this, std::placeholders::_1));
            }
        }
        srv_save_map = nh->create_service<estimate_msgs::srv::SaveMap>("save_map",
            std::bind(&RESPLE::savePCDCallback, this, std::placeholders::_1, std::placeholders::_2));
        logStartupInfo(nh);
    }

    void processData()
    {
        rclcpp::Rate rate(20);
        int64_t max_spl_knots = 0;
        int64_t t_last_map_upd = 0;
        while (true) {      
            for (auto& [lidar_name, lidar_data] : lidars_data) {
                while (!lidar_data.t_buff.empty()) {
                    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_frame(new pcl::PointCloud<pcl::PointXYZINormal>());
                    int64_t time_begin;
                    {
                        std::lock_guard<std::mutex> lock(lidar_data.mtx_pc);
                        pc_frame->points = lidar_data.pc_buff.front();
                        lidar_data.pc_buff.pop_front();
                        time_begin = lidar_data.t_buff.front();
                        lidar_data.t_buff.pop_front();
                    }
                    std::vector<int> indices;
                    pcl::removeNaNFromPointCloud(*pc_frame, *pc_frame, indices);
                    pc_last_ds->clear();

                    ds_filter_body.setInputCloud(pc_frame);
                    ds_filter_body.filter(*pc_last_ds);
                    sort(pc_last_ds->points.begin(), pc_last_ds->points.end(), &CommonUtils::time_list);
                    const LidarConfig& lidar = lidars.at(lidar_name);
                    for (size_t i = 0; i < pc_last_ds->points.size(); i++) {
                        PointData pt(pc_last_ds->points[i], time_begin, lidar.q_bl, lidar.t_bl, lidar.w_pt);
                        lidar_data.pt_buff.push_back(pt);
                    }
                }
            }            
            if (!if_lidar_only && !imu_int_buff.empty()) {
                Eigen::aligned_vector<sensor_msgs::msg::Imu::SharedPtr> imu_buff_msg;
                {
                    std::lock_guard<std::mutex> lock(m_buff);
                    imu_buff_msg = imu_int_buff;
                    imu_int_buff.clear();
                }
                for (size_t i = 0; i < imu_buff_msg.size(); i++) {
                    const auto imu_msg = imu_buff_msg[i];
                    int64_t t_ns = rclcpp::Time(imu_msg->header.stamp).nanoseconds();
                    Eigen::Vector3d acc(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
                    if (acc_ratio) acc *= CommonUtils::kGravity;
                    Eigen::Vector3d gyro(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
                    ImuData imu(t_ns, gyro, acc); 
                    imu_buff.push_back(imu);
                }
            }
            if(!initialization()) {
                rate.sleep();
                continue;
            }
            while (collectMeasurements()) {
                int64_t max_time_ns = pt_meas.back().time_ns;
                if (wheel_cfg.enable) {
                    while (!wheel_meas.empty() && wheel_meas.front().time_ns < spline->maxTimeNs() - spline->getKnotTimeIntervalNs()) {
                        wheel_meas.pop_front();
                    }
                }
                if (if_lidar_only) {
                    estimator_lo.propRCP(max_time_ns);
                    estimator_lo.updateIEKFLiDAR(pt_meas, &ikdtree, param.nn_thresh, param.coeff_cov, wheel_meas, wheel_cfg, param.num_nn);
                } else {
                    if (!imu_meas.empty()) {
                        max_time_ns = std::max(imu_meas.back().time_ns, max_time_ns);
                    }
                    while (!imu_meas.empty() && imu_meas.front().time_ns < spline->maxTimeNs() - spline->getKnotTimeIntervalNs()) {
                        imu_meas.pop_front();
                    }
                    estimator_lio.propRCP(max_time_ns);
                    estimator_lio.updateIEKFLiDARInertial(pt_meas, &ikdtree, param.nn_thresh, imu_meas, gravity, param.cov_acc, param.cov_gyro, param.coeff_cov, wheel_meas, wheel_cfg, param.num_nn);
                }
                if (wheel_cfg.enable && !wheel_meas.empty()) {
                    const WheelData& w_dbg = wheel_meas.back();
                    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("RESPLE"), wheel_log_clock_, 1000,
                        "wheel odom: meas=[%.3f %.3f %.3f] pred=[%.3f %.3f %.3f]",
                        w_dbg.vel.x(), w_dbg.vel.y(), w_dbg.vel.z(), w_dbg.vel_itp.x(), w_dbg.vel_itp.y(), w_dbg.vel_itp.z());
                }
                #pragma omp parallel for num_threads(kNumOmpThreads)
                for (size_t i = 0; i < pt_meas.size(); i++) {
                    PointData& pt_data = pt_meas[i];            
                    Association::pointBodyToWorld(pt_data.time_ns, spline, pt_data.pt, pt_data.pt_w, pt_data.t_bl, pt_data.q_bl);
                }            
                for (size_t i = 0; i < pt_meas.size(); i++) {
                    PointData& pt_data = pt_meas[i];
                    pc_world.points.push_back(pt_data.pt_w);
                    accum_nearest_points.push_back(pt_data.nearest_points);
                }
                pt_meas.clear();
                if (spline->numKnots() > max_spl_knots) {
                    estimate_msgs::msg::Spline spline_msg;
                    spline->getSplineMsg(spline_msg, std::max(int(max_spl_knots-1),0));
                    estimate_msgs::msg::Estimate est_msg;
                    est_msg.spline = spline_msg;
                    est_msg.if_full_window.data = (spline->numKnots() >= 4);
                    est_msg.runtime.data = 0;
                    pub_est->publish(est_msg);  
                    max_spl_knots = spline->numKnots();       
                }
                if (max_time_ns >= t_last_map_upd + kMapUpdatePeriodNs) {
                    mapIncremental();
                    publishFrameWorld();
                    lasermapFovSegment();
                    pc_world.clear();
                    accum_nearest_points.clear();
                    t_last_map_upd = max_time_ns;
                }
            }                      
        }
    }    

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

    std::string node_name = "RESPLE";
    rclcpp::Service<estimate_msgs::srv::SaveMap>::SharedPtr srv_save_map;
    std::string pcd_save_path;
    std::mutex mtx_map;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_ouster;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_livox2;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_hesai;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_livox_mid360;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cur_scan;
    rclcpp::Publisher<estimate_msgs::msg::Estimate>::SharedPtr pub_est;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_start_time;
    std::shared_ptr<tf2_ros::TransformBroadcaster> br;
    const std::string frame_id = "body";
    std::string odom_id = "world";

    std::map<std::string, LidarConfig> lidars;
    float ds_lm_voxel;
    pcl::VoxelGrid<pcl::PointXYZINormal> ds_filter_body;    
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last_ds;
    pcl::PointCloud<pcl::PointXYZINormal> pc_world;
    int point_filter_num = 1;
    int64_t time_offset = 0;

    std::vector<BoxPointType> cub_needrm;
    BoxPointType LocalMap_Points;
    std::vector<Eigen::aligned_vector<pcl::PointXYZINormal>> accum_nearest_points;
    double cube_len = 2000;
    const float MOV_THRESHOLD = 1.5f;
    // Incremental local-map update cadence (mapIncremental/publishFrameWorld/lasermapFovSegment).
    static constexpr int64_t kMapUpdatePeriodNs = 100'000'000;
    // Width of the initial time window (from the first point's timestamp) whose
    // points seed the very first local map build in initialization().
    static constexpr int64_t kInitialMapWindowNs = 100'000'000;
    float det_range = 100.0;
    bool if_init_map = false;
    struct LidarData {
        Eigen::aligned_deque<Eigen::aligned_vector<pcl::PointXYZINormal>> pc_buff;
        std::deque<int64_t> t_buff;
        std::mutex mtx_pc;
        Eigen::aligned_deque<PointData> pt_buff;
    };
    std::map<std::string, LidarData> lidars_data;    
    Eigen::aligned_deque<PointData> pt_meas;    

    bool if_lidar_only;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    Eigen::aligned_deque<ImuData> imu_buff;
    Eigen::aligned_deque<ImuData> imu_meas;
    Eigen::aligned_vector<sensor_msgs::msg::Imu::SharedPtr> imu_int_buff;    
    std::mutex m_buff;
    bool acc_ratio;
    Eigen::Vector3d cov_ba;
    Eigen::Vector3d cov_bg;
    Eigen::Vector3d gravity;

    WheelConfig wheel_cfg;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_wheel_odom;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_wheel_twist;
    Eigen::aligned_deque<WheelData> wheel_buff_;
    Eigen::aligned_deque<WheelData> wheel_meas;
    std::mutex m_wheel_buff;
    rclcpp::Clock wheel_log_clock_{RCL_STEADY_TIME};

    RelocConfig reloc_cfg;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr prior_map_cloud;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_prior_map;
    std::mutex m_pose_guess;
    bool if_have_pose_guess = false;
    Eigen::Vector3d pose_guess_t;
    Eigen::Quaterniond pose_guess_q;

    bool if_init_filter = false;
    Estimator<24> estimator_lo;
    Estimator<30> estimator_lio;
    SplineState* spline;
    double cov_P0 = 0.02;
    double cov_RCP_pos_old = 0.02;
    double cov_RCP_ort_old = 0.02;
    double cov_RCP_pos_new = 0.1;    
    double cov_RCP_ort_new = 0.1;    
    double cov_sys_pos = 0.1;    
    double cov_sys_ort = 0.01;    
    Parameters param;
    int64_t dt_ns;
    int num_points_upd;
    
    const std::string baselink_frame = "base_link";
    const std::string odom_frame = "odom";

    void readParameters(rclcpp::Node::SharedPtr &nh)
    {
        odom_id = CommonUtils::readParam<std::string>(nh, "frames.world", std::string("world"));
        ds_lm_voxel = CommonUtils::readParam<float>(nh, "mapping.ds_lm_voxel");
        float ds_scan_voxel = CommonUtils::readParam<float>(nh, "mapping.ds_scan_voxel");
        ds_filter_body.setLeafSize(ds_scan_voxel, ds_scan_voxel, ds_scan_voxel);
        param.nn_thresh = CommonUtils::readParam<double>(nh, "mapping.nn_thresh");
        if_lidar_only = CommonUtils::readParam<bool>(nh, "if_lidar_only");
        if (!if_lidar_only) {
            acc_ratio = CommonUtils::readParam<bool>(nh, "imu.acc_ratio");
            std::vector<double> bias_acc_var = CommonUtils::readParam<std::vector<double>>(nh, "imu.cov_ba");
            cov_ba << bias_acc_var.at(0), bias_acc_var.at(1), bias_acc_var.at(2);
            std::vector<double> bias_gyro_var = CommonUtils::readParam<std::vector<double>>(nh, "imu.cov_bg");
            cov_bg << bias_gyro_var.at(0), bias_gyro_var.at(1), bias_gyro_var.at(2);
            std::vector<double> acc_var = CommonUtils::readParam<std::vector<double>>(nh, "imu.cov_acc");
            param.cov_acc << acc_var.at(0), acc_var.at(1), acc_var.at(2);
            std::vector<double> gyro_var = CommonUtils::readParam<std::vector<double>>(nh, "imu.cov_gyro");
            param.cov_gyro << gyro_var.at(0), gyro_var.at(1), gyro_var.at(2);
        }

        wheel_cfg = WheelConfig(nh);
        reloc_cfg = RelocConfig(nh);

        dt_ns = 1e9 / CommonUtils::readParam<int>(nh, "spline.knot_hz");
        double dt_s = double(dt_ns) * 1e-9;
        cov_P0 = CommonUtils::readParam<double>(nh, "spline.cov_p0");
        cov_P0 *= (dt_s*dt_s);
        cov_RCP_pos_old = CommonUtils::readParam<double>(nh, "spline.cov_rcp_pos_old");
        cov_RCP_ort_old = CommonUtils::readParam<double>(nh, "spline.cov_rcp_ort_old");
        cov_RCP_pos_new = CommonUtils::readParam<double>(nh, "spline.cov_rcp_pos_new");
        cov_RCP_ort_new = CommonUtils::readParam<double>(nh, "spline.cov_rcp_ort_new");
        double std_pos = CommonUtils::readParam<double>(nh, "spline.std_sys_pos");
        double std_ort = CommonUtils::readParam<double>(nh, "spline.std_sys_ort");
        cov_sys_pos = std_pos*std_pos*dt_s*dt_s;
        cov_sys_ort = std_ort*std_ort*dt_s*dt_s;
        param.coeff_cov = CommonUtils::readParam<double>(nh, "mapping.coeff_cov", 10);

        cube_len = CommonUtils::readParam<double>(nh, "mapping.cube_len");
        point_filter_num = CommonUtils::readParam<int>(nh, "mapping.point_filter_num");
        num_points_upd = CommonUtils::readParam<int>(nh, "mapping.num_points_upd");
        if (if_lidar_only) {
            estimator_lo.n_iter = CommonUtils::readParam<int>(nh, "spline.n_iter");
        } else {
            estimator_lio.n_iter = CommonUtils::readParam<int>(nh, "spline.n_iter");
        }
        pc_last.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
        pc_last_ds.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
        param.num_nn = CommonUtils::readParam<int>(nh, "mapping.num_nn", 5);
        double lidar_time_offset = CommonUtils::readParam<double>(nh, "lidar.time_offset", 0.0);
        time_offset = 1e9*lidar_time_offset;
        pcd_save_path = CommonUtils::readParam<std::string>(nh, "mapping.pcd_save_path", std::string("/tmp/resple_map.pcd"));
    }

    // RCLCPP_INFO (not std::cout): under ros2 launch, this node's raw stdout does not
    // reliably reach the console, but ROS logging does. Visible whenever this node's
    // log level is INFO or more verbose (the launch file's --log-level argument).
    void logStartupInfo(rclcpp::Node::SharedPtr& nh)
    {
        RCLCPP_INFO_STREAM(nh->get_logger(), "========== RESPLE initializing ==========");
        RCLCPP_INFO_STREAM(nh->get_logger(), "Mode: " << (if_lidar_only ? "LiDAR-only" : "LiDAR-Inertial"));
        if (!if_lidar_only) {
            RCLCPP_INFO_STREAM(nh->get_logger(), "IMU topic: " << sub_imu->get_topic_name());
        }
        RCLCPP_INFO_STREAM(nh->get_logger(), "Spline knot rate: " << (1000000000LL / dt_ns) << " Hz");
        RCLCPP_INFO_STREAM(nh->get_logger(), "LiDAR sensors (" << lidars.size() << "):");
        for (const auto& [name, lidar] : lidars) {
            RCLCPP_INFO_STREAM(nh->get_logger(), "  - " << name << ": type=" << lidar.type << " topic=" << lidar.topic
                       << " scan_line=" << lidar.scan_line << " blind=" << lidar.blind << "m");
        }
        if (wheel_cfg.enable) {
            RCLCPP_INFO_STREAM(nh->get_logger(), "Wheel odometry: enabled, topic=" << wheel_cfg.topic << " (" << wheel_cfg.topic_type
                       << "), use_only_vx=" << (wheel_cfg.use_only_vx ? "true" : "false"));
        } else {
            RCLCPP_INFO_STREAM(nh->get_logger(), "Wheel odometry: disabled");
        }
        RCLCPP_INFO_STREAM(nh->get_logger(), "Local map: cube_len=" << cube_len << "m ds_lm_voxel=" << ds_lm_voxel << "m");
        if (reloc_cfg.enable) {
            RCLCPP_INFO_STREAM(nh->get_logger(), "Relocalization: enabled, map=" << reloc_cfg.map_path
                       << ", guess=" << (reloc_cfg.initial_guess ? "config t0/q0" : "waiting for /initialpose"));
        } else {
            RCLCPP_INFO_STREAM(nh->get_logger(), "Relocalization: disabled");
        }
        RCLCPP_INFO_STREAM(nh->get_logger(), "==========================================");
    }

    void initFilter(int64_t start_t_ns, Eigen::Vector3d t_init = Eigen::Vector3d::Zero(), Eigen::Quaterniond q_init = Eigen::Quaterniond::Identity())
    {
        Eigen::Matrix<double, 24, 24> cov_RCPs = cov_P0 * Eigen::Matrix<double, 24, 24>::Identity();
        Eigen::Matrix<double, 30, 30> Q = Eigen::Matrix<double, 30, 30>::Zero();
        Eigen::Matrix<double, 6, 6> Q_block_old = Eigen::Matrix<double, 6, 6>::Zero();
        Q_block_old.topLeftCorner<3, 3>() = cov_RCP_pos_old*cov_sys_pos *Eigen::Matrix3d::Identity();
        Q_block_old.bottomRightCorner<3, 3>() = cov_RCP_ort_old*cov_sys_ort *Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 6, 6> Q_block_new = Eigen::Matrix<double, 6, 6>::Zero();
        Q_block_new.topLeftCorner<3, 3>() = cov_RCP_pos_new*cov_sys_pos *Eigen::Matrix3d::Identity();
        Q_block_new.bottomRightCorner<3, 3>() = cov_RCP_ort_new*cov_sys_ort *Eigen::Matrix3d::Identity();        
        Q.topLeftCorner<6, 6>() = Q_block_old;
        Q.block<6, 6>(6, 6) = Q_block_old;
        Q.block<6, 6>(12, 12) = Q_block_old;
        Q.bottomRightCorner<6, 6>() = Q_block_new;  
        if (if_lidar_only) {
            estimator_lo.setState(dt_ns, start_t_ns, t_init, q_init, Q.topLeftCorner<24, 24>(), cov_RCPs);  
            spline = estimator_lo.getSpline();
        } else {
            Eigen::Matrix<double, 30, 30> cov_x = Eigen::Matrix<double, 30, 30>::Zero();
            cov_x.topLeftCorner<24, 24>() = cov_RCPs;
            cov_x.block<3, 3>(24, 24) = cov_ba.asDiagonal();
            cov_x.block<3, 3>(27, 27) = cov_bg.asDiagonal();            
            estimator_lio.setState(dt_ns, start_t_ns, t_init, q_init, Q, cov_x);   
            spline = estimator_lio.getSpline(); 
        }
    }     

    void getImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        std::lock_guard<std::mutex> lock(m_buff);
        imu_int_buff.push_back(imu_msg);
    }

    void getWheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        int64_t t_ns = rclcpp::Time(odom_msg->header.stamp).nanoseconds();
        Eigen::Vector3d vel(odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y, odom_msg->twist.twist.linear.z);
        std::lock_guard<std::mutex> lock(m_wheel_buff);
        wheel_buff_.emplace_back(t_ns, vel);
    }

    void getWheelTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg)
    {
        int64_t t_ns = rclcpp::Time(twist_msg->header.stamp).nanoseconds();
        Eigen::Vector3d vel(twist_msg->twist.linear.x, twist_msg->twist.linear.y, twist_msg->twist.linear.z);
        std::lock_guard<std::mutex> lock(m_wheel_buff);
        wheel_buff_.emplace_back(t_ns, vel);
    }

    // RViz's "2D Pose Estimate" tool (already wired to /initialpose in config.rviz)
    // publishes here. Only the first message is used (v1 has no hot re-trigger);
    // it only carries x/y/yaw (z/roll/pitch are always 0) — ICP refines the rest.
    void getInitialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_pose_guess);
        if (if_have_pose_guess) {
            return;
        }
        pose_guess_t = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        pose_guess_q = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        if_have_pose_guess = true;
        RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Relocalization: received initial pose guess from /initialpose");
    }

    // Shared by all 4 LiDAR types (see utils/lidar_adapters.h): Adapter absorbs the
    // per-type message layout/timestamp convention/structural filter; decimation,
    // blind-range filtering and cross-scan monotonic-time dedup are generic here.
    template<typename Adapter>
    void genericLidarCallback(const typename Adapter::Msg::SharedPtr msg_in)
    {
        const LidarConfig& lidar = lidars.at(Adapter::kTypeName);
        rclcpp::Time stamp_begin(msg_in->header.stamp);
        int64_t time_begin = stamp_begin.nanoseconds() - time_offset;
        Adapter adapter(*msg_in, lidar, stamp_begin);
        size_t plsize = adapter.size();
        if (plsize == 0) return;

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());
        pc_last->reserve(plsize);
        static int64_t last_t_ns = time_begin;
        int64_t max_ofs_ns = 0;
        int valid_count = 0;
        float blind = lidar.blind;
        for (size_t i = adapter.startIndex(); i < plsize; i++) {
            pcl::PointXYZINormal pt;
            if (!adapter.get(i, pt)) continue;
            if ((valid_count++ % point_filter_num) != 0) continue;
            int64_t ofs_ns = CommonUtils::ms2ns(pt.intensity);
            if (pt.intensity >= 0 && pt.x*pt.x+pt.y*pt.y+pt.z*pt.z > (blind * blind) && ofs_ns + time_begin > last_t_ns) {
                max_ofs_ns = std::max(max_ofs_ns, ofs_ns);
                pc_last->points.push_back(pt);
            }
        }
        LidarData& lidar_buffs = lidars_data.at(Adapter::kTypeName);
        {
            std::lock_guard<std::mutex> lock(lidar_buffs.mtx_pc);
            lidar_buffs.pc_buff.push_back(pc_last->points);
            lidar_buffs.t_buff.push_back(time_begin);
        }
        last_t_ns = time_begin + max_ofs_ns;
    }

    void publishFrameWorld() 
    {
        int size = pc_world.points.size();
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudWorld(new pcl::PointCloud<pcl::PointXYZI>(size, 1));
        for (int i = 0; i < size; i++) {
            laserCloudWorld->points[i].x = pc_world.points[i].x;
            laserCloudWorld->points[i].y = pc_world.points[i].y;
            laserCloudWorld->points[i].z = pc_world.points[i].z;
            laserCloudWorld->points[i].intensity = pc_world.points[i].curvature; 
        }
        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = rclcpp::Time(spline->maxTimeNs());
        laserCloudmsg.header.frame_id = odom_id;
        pub_cur_scan->publish(laserCloudmsg);
    }   

    // Averages the first ~15 buffered IMU samples up to start_t_ns into a body-frame
    // gravity vector (magnitude CommonUtils::kGravity), popping consumed samples from
    // imu_buff same as always. Shared by the non-reloc filter init below (which derives
    // a gravity-aligned q_WI from it) and initializeFromPriorMap (which instead rotates
    // it by the ICP-refined orientation, since the prior map's world frame is already
    // gravity-aligned by construction).
    Eigen::Vector3d averageGravityBody(int64_t start_t_ns)
    {
        Eigen::Vector3d gravity_sum(0, 0, 0);
        int n_imu;
        {
            std::lock_guard<std::mutex> lock(m_buff);
            int buff_size = imu_buff.size();
            n_imu = std::min(15, buff_size);
            for (int i = 0; i < n_imu; i++) {
                gravity_sum += imu_buff.at(i).accel;
            }
            while (!imu_buff.empty() && imu_buff.front().time_ns < start_t_ns) {
                imu_buff.pop_front();
            }
        }
        gravity_sum /= n_imu;
        return gravity_sum.normalized() * CommonUtils::kGravity;
    }

    bool initialization()
    {
        if (if_init_filter && if_init_map) {
            return true;
        }
        for (const auto& [lidar_name, lidar_data] : lidars_data) {
            if (lidar_data.pt_buff.empty()) {
                return false;
            }
        }
        int64_t start_t_ns = std::numeric_limits<int64_t>::max();
        for (const auto& [lidar_name, lidar_data] : lidars_data) {
            start_t_ns = std::min(start_t_ns, std::max(lidar_data.pt_buff.front().time_ns, int64_t(0)));
        }
        if (reloc_cfg.enable) {
            return initializeFromPriorMap(start_t_ns);
        }
        if (!if_init_filter) {
            Eigen::Quaterniond q_WI = Eigen::Quaterniond::Identity();
            if (!if_lidar_only) {
                Eigen::Vector3d gravity_ave = averageGravityBody(start_t_ns);
                Eigen::Matrix3d R0 = CommonUtils::g2R(gravity_ave);
                double yaw = CommonUtils::R2ypr(R0).x();
                R0 = CommonUtils::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
                Eigen::Quaterniond q0(R0);
                q_WI = Quater::positify(q0);
                gravity = q_WI * gravity_ave;
            }
            initFilter(start_t_ns, Eigen::Vector3d(0, 0, 0), q_WI);
            if_init_filter = true;            
            std_msgs::msg::Int64 start_time;
            start_time.data = start_t_ns;
            pub_start_time->publish(start_time);    
        }
        if (!if_init_map) {
            if(ikdtree.Root_Node == nullptr) {
                ikdtree.set_downsample_param(ds_lm_voxel);
            }
            if (if_lidar_only) {
                estimator_lo.propRCP(start_t_ns);  
            } else {
                estimator_lio.propRCP(start_t_ns);
            }
            int feats_down_size = 0;
            for (const auto& [lidar_name, lidar_data] : lidars_data) {
                for (size_t i = 0; i < lidar_data.pt_buff.size(); i++) {
                    if (lidar_data.pt_buff[i].time_ns < start_t_ns + kInitialMapWindowNs) {
                        feats_down_size++;
                    } else {
                        break;
                    }
                }
            }            
            if(feats_down_size < 100) {
                return false;
            }            
            pc_world.clear();
            pc_world.resize(feats_down_size); 
            int world_i = 0;
            for (const auto& [lidar_name, lidar_data] : lidars_data) {
                for (size_t i = 0; i < lidar_data.pt_buff.size(); i++) {
                    if (lidar_data.pt_buff[i].time_ns < start_t_ns + kInitialMapWindowNs) {
                        Association::pointBodyToWorld(start_t_ns, spline, lidar_data.pt_buff[i].pt,
                            pc_world.points[world_i], lidar_data.pt_buff[i].t_bl, lidar_data.pt_buff[i].q_bl);
                        world_i++;
                    } else {
                        break;
                    }
                }
            }
            for (auto& [lidar_name, lidar_data] : lidars_data) {
                while (!lidar_data.pt_buff.empty() && lidar_data.pt_buff.front().time_ns < start_t_ns + kInitialMapWindowNs) {
                    lidar_data.pt_buff.pop_front();
                }
            }            
            ikdtree.Build(pc_world.points);
            pc_world.clear();
            if_init_map = true;
        }
        return false;
    }

    // Relocalization-enabled counterpart to the filter-init/map-build steps above:
    // waits for a pose guess (config or /initialpose), refines it via ICP against the
    // loaded prior map, then seeds initFilter/ikdtree from the refined pose/prior map
    // instead of building from scratch. Same one-shot, "seed everything then return
    // false" convention as the non-reloc path (next tick's if_init_filter && if_init_map
    // check returns true).
    bool initializeFromPriorMap(int64_t start_t_ns)
    {
        Eigen::Vector3d t_guess;
        Eigen::Quaterniond q_guess;
        {
            std::lock_guard<std::mutex> lock(m_pose_guess);
            if (!if_have_pose_guess) {
                return false;
            }
            t_guess = pose_guess_t;
            q_guess = pose_guess_q;
        }

        int feats_down_size = 0;
        for (const auto& [lidar_name, lidar_data] : lidars_data) {
            for (size_t i = 0; i < lidar_data.pt_buff.size(); i++) {
                if (lidar_data.pt_buff[i].time_ns < start_t_ns + kInitialMapWindowNs) {
                    feats_down_size++;
                } else {
                    break;
                }
            }
        }
        if (feats_down_size < 100) {
            return false;
        }

        pc_world.clear();
        pc_world.resize(feats_down_size);
        int world_i = 0;
        for (const auto& [lidar_name, lidar_data] : lidars_data) {
            for (size_t i = 0; i < lidar_data.pt_buff.size(); i++) {
                if (lidar_data.pt_buff[i].time_ns < start_t_ns + kInitialMapWindowNs) {
                    Relocalization::pointSensorToBody(lidar_data.pt_buff[i].pt, pc_world.points[world_i],
                        lidar_data.pt_buff[i].t_bl, lidar_data.pt_buff[i].q_bl);
                    world_i++;
                } else {
                    break;
                }
            }
        }
        for (auto& [lidar_name, lidar_data] : lidars_data) {
            while (!lidar_data.pt_buff.empty() && lidar_data.pt_buff.front().time_ns < start_t_ns + kInitialMapWindowNs) {
                lidar_data.pt_buff.pop_front();
            }
        }
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_body(new pcl::PointCloud<pcl::PointXYZINormal>(pc_world));
        pc_world.clear();

        Eigen::Vector3d t_refined;
        Eigen::Quaterniond q_refined;
        double fitness;
        if (!Relocalization::refineInitialPose(source_body, prior_map_cloud, t_guess, q_guess, reloc_cfg,
                t_refined, q_refined, fitness)) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(node_name), "Relocalization: ICP failed to converge or fitness ("
                << fitness << ") exceeded threshold (" << reloc_cfg.icp_fitness_threshold
                << ") -- falling back to normal from-scratch initialization for the rest of this run");
            reloc_cfg.enable = false;
            return false;
        }
        RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Relocalization: ICP converged, fitness=" << fitness);

        if (!if_lidar_only) {
            gravity = q_refined * averageGravityBody(start_t_ns);
        }

        initFilter(start_t_ns, t_refined, q_refined);
        if_init_filter = true;
        std_msgs::msg::Int64 start_time;
        start_time.data = start_t_ns;
        pub_start_time->publish(start_time);

        if (if_lidar_only) {
            estimator_lo.propRCP(start_t_ns);
        } else {
            estimator_lio.propRCP(start_t_ns);
        }

        if (ikdtree.Root_Node == nullptr) {
            ikdtree.set_downsample_param(ds_lm_voxel);
        }
        ikdtree.Build(prior_map_cloud->points);
        if_init_map = true;
        return false;
    }

    bool collectMeasurements()
    {
        rclcpp::Rate rate(20);
        int64_t pt_min_time = std::numeric_limits<int64_t>::max();
        int64_t pt_max_time = std::numeric_limits<int64_t>::max();            
        for (const auto& [lidar_name, lidar_data] : lidars_data) {
            if (lidar_data.pt_buff.empty()) {
                rate.sleep();
                return false;
            }
            pt_min_time = std::min(pt_min_time, lidar_data.pt_buff.front().time_ns);
            pt_max_time = std::min(pt_max_time, lidar_data.pt_buff.back().time_ns);
        }            
        if (pt_max_time <= spline->maxTimeNs() + dt_ns) {
            rate.sleep();
            return false;
        }      
        if (!if_lidar_only && (imu_buff.empty() || imu_buff.back().time_ns <= spline->maxTimeNs())) {
            rate.sleep();
            return false;
        }                   
        int64_t max_time_ns = std::min(spline->maxTimeNs(), pt_min_time + dt_ns);
        if (pt_min_time > max_time_ns) {
            if (if_lidar_only) {
                estimator_lo.propRCP(pt_min_time);
            } else {
                estimator_lio.propRCP(pt_min_time);
            }                
            max_time_ns = spline->maxTimeNs();
        }     
        if (spline->numKnots() > 4) {
            max_time_ns = spline->maxTimeNs();
        }
        int cnt = 0;
        for (auto& [lidar_name, lidar_data] : lidars_data) {
            while (!lidar_data.pt_buff.empty() && lidar_data.pt_buff.front().time_ns <= max_time_ns &&
                    cnt < num_points_upd) {
                if (spline->numKnots() < 10 || lidar_data.pt_buff.front().time_ns >= spline->maxTimeNs() - dt_ns) {
                    pt_meas.emplace_back(lidar_data.pt_buff.front());
                }
                lidar_data.pt_buff.pop_front();
                cnt++;
            }
        }                    
        if (!if_lidar_only) {
            while (!imu_buff.empty() && imu_buff.front().time_ns < spline->minTimeNs()) {
                imu_buff.pop_front();
            }
            while (!imu_buff.empty() && imu_buff.front().time_ns <= max_time_ns) {
                imu_meas.emplace_back(imu_buff.front());
                imu_buff.pop_front();
            }
        }
        if (wheel_cfg.enable) {
            std::lock_guard<std::mutex> lock(m_wheel_buff);
            while (!wheel_buff_.empty() && wheel_buff_.front().time_ns < spline->minTimeNs()) {
                wheel_buff_.pop_front();
            }
            while (!wheel_buff_.empty() && wheel_buff_.front().time_ns <= max_time_ns) {
                wheel_meas.emplace_back(wheel_buff_.front());
                wheel_buff_.pop_front();
            }
        }
        return true;

    }

    Eigen::Vector3d getPositionLiDAR(int64_t t_ns, const Eigen::Vector3d& t_bl)
    {
        if (if_lidar_only) {
            estimator_lo.propRCP(t_ns);
        } else {
            estimator_lio.propRCP(t_ns);
        }
        Eigen::Quaterniond orient_interp;
        Eigen::Vector3d t_interp = spline->itpPosition(t_ns);
        spline->itpQuaternion(t_ns, &orient_interp);
        Eigen::Vector3d t = orient_interp * t_bl + t_interp;
        return t;
    }       

    void lasermapFovSegment()
    {
        static bool Localmap_Initialized = false;
        cub_needrm.shrink_to_fit();
        Eigen::Vector3d pos_lidar_min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max());
        Eigen::Vector3d pos_lidar_max(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
                std::numeric_limits<double>::lowest());
        for (const auto& [lidar_name, lidar] : lidars) {
            Eigen::Vector3d pos_lidar = getPositionLiDAR(spline->maxTimeNs(), lidar.t_bl);
            pos_lidar_min = pos_lidar_min.array().min(pos_lidar.array()).matrix();
            pos_lidar_max = pos_lidar_max.array().max(pos_lidar.array()).matrix();
        }        
        if (!Localmap_Initialized){
            for (int i = 0; i < 3; i++){
                LocalMap_Points.vertex_min[i] = pos_lidar_min(i) - cube_len / 2.0;
                LocalMap_Points.vertex_max[i] = pos_lidar_max(i) + cube_len / 2.0;                
            }
            Localmap_Initialized = true;
            return;
        }
        float dist_to_map_edge[3][2];
        bool need_move = false;
        for (int i = 0; i < 3; i++){
            dist_to_map_edge[i][0] = fabs(pos_lidar_min(i) - LocalMap_Points.vertex_min[i]);
            dist_to_map_edge[i][1] = fabs(pos_lidar_max(i) - LocalMap_Points.vertex_max[i]);            
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * det_range || dist_to_map_edge[i][1] <= MOV_THRESHOLD * det_range) need_move = true;
        }
        if (!need_move) return;
        BoxPointType New_LocalMap_Points, tmp_boxpoints;
        New_LocalMap_Points = LocalMap_Points;
        float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * det_range) * 0.5 * 0.9, double(det_range * (MOV_THRESHOLD -1)));
        for (int i = 0; i < 3; i++){
            tmp_boxpoints = LocalMap_Points;
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * det_range){
                New_LocalMap_Points.vertex_max[i] -= mov_dist;
                New_LocalMap_Points.vertex_min[i] -= mov_dist;
                tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
                cub_needrm.emplace_back(tmp_boxpoints);
            } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * det_range){
                New_LocalMap_Points.vertex_max[i] += mov_dist;
                New_LocalMap_Points.vertex_min[i] += mov_dist;
                tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
                cub_needrm.emplace_back(tmp_boxpoints);
            }
        }
        LocalMap_Points = New_LocalMap_Points;

        if(cub_needrm.size() > 0) {
            std::lock_guard<std::mutex> lock(mtx_map);
            ikdtree.Delete_Point_Boxes(cub_needrm);
        }
    }

    void mapIncremental()
    {
        // Half a voxel cell's space diagonal (sqrt(3)/2): if the nearest already-
        // mapped point is farther than this from the candidate's voxel center, the
        // candidate is in a different, not-yet-downsampled voxel and is kept as-is.
        constexpr double kHalfVoxelDiagonal = 0.8660254037844386;

        Eigen::aligned_vector<pcl::PointXYZINormal> PointToAdd;
        Eigen::aligned_vector<pcl::PointXYZINormal> PointNoNeedDownsample;
        int feats_down_size = pc_world.points.size();
        PointToAdd.reserve(feats_down_size);
        PointNoNeedDownsample.reserve(feats_down_size);
        for(int i = 0; i < feats_down_size; i++) {     
            const pcl::PointXYZINormal& point = pc_world.points[i];       
            if (!accum_nearest_points[i].empty()) {
                const Eigen::aligned_vector<pcl::PointXYZINormal> &points_near = accum_nearest_points[i];
                bool need_add = true;
                pcl::PointXYZINormal downsample_result, mid_point; 
                
                mid_point.x = floor(point.x/ds_lm_voxel)*ds_lm_voxel + 0.5 * ds_lm_voxel;
                mid_point.y = floor(point.y/ds_lm_voxel)*ds_lm_voxel + 0.5 * ds_lm_voxel;
                mid_point.z = floor(point.z/ds_lm_voxel)*ds_lm_voxel + 0.5 * ds_lm_voxel;
                if (fabs(points_near[0].x - mid_point.x) > kHalfVoxelDiagonal * ds_lm_voxel || fabs(points_near[0].y - mid_point.y) > kHalfVoxelDiagonal * ds_lm_voxel || fabs(points_near[0].z - mid_point.z) > kHalfVoxelDiagonal * ds_lm_voxel){
                    PointNoNeedDownsample.emplace_back(pc_world.points[i]);
                    continue;
                }
                for (size_t readd_i = 0; readd_i < points_near.size(); readd_i ++) {
                    if (fabs(points_near[readd_i].x - mid_point.x) < 0.5 * ds_lm_voxel && fabs(points_near[readd_i].y - mid_point.y) < 0.5 * ds_lm_voxel && fabs(points_near[readd_i].z - mid_point.z) < 0.5 * ds_lm_voxel) {
                        need_add = false;
                        break;
                    }
                }
                if (need_add) PointToAdd.emplace_back(point);
            } else {
                PointNoNeedDownsample.emplace_back(point);
            }
        }
        std::lock_guard<std::mutex> lock(mtx_map);
        ikdtree.Add_Points(PointToAdd, true);
        ikdtree.Add_Points(PointNoNeedDownsample, false);
    }

    void savePCDCallback(const std::shared_ptr<estimate_msgs::srv::SaveMap::Request> request,
                         std::shared_ptr<estimate_msgs::srv::SaveMap::Response> response)
    {
        const std::string& save_path = request->path.empty() ? pcd_save_path : request->path;
        Eigen::aligned_vector<pcl::PointXYZINormal> map_points;
        {
            std::lock_guard<std::mutex> lock(mtx_map);
            ikdtree.flatten(ikdtree.Root_Node, map_points, NOT_RECORD);
        }
        pcl::PointCloud<pcl::PointXYZINormal> map_cloud;
        map_cloud.points = map_points;
        map_cloud.width = map_cloud.points.size();
        map_cloud.height = 1;
        map_cloud.is_dense = true;
        int ret = pcl::io::savePCDFileBinary(save_path, map_cloud);
        response->success = (ret == 0);
        response->message = response->success
            ? "Saved map to " + save_path + " (" + std::to_string(map_cloud.size()) + " points)"
            : "Failed to save map to " + save_path;
        RCLCPP_INFO(rclcpp::get_logger(node_name), "%s", response->message.c_str());
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("RESPLE");
    RESPLE resple(nh);
    RCLCPP_INFO_STREAM(nh->get_logger(), "RESPLE starts!");
    rclcpp::Rate rate(200);
    std::thread opt{&RESPLE::processData, &resple};
    while (rclcpp::ok()) {
        rclcpp::spin_some(nh);
        rate.sleep();
    }
    opt.join();
    rclcpp::shutdown();
}
