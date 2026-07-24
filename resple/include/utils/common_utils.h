#pragma once

#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <filesystem>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <omp.h>

#include "utils/eigen_utils.hpp"

// OpenMP worker count for the parallel loops in Estimator.h/Association.h.
// Not runtime-configurable (unlike mapping.num_nn, threaded explicitly via
// Parameters::num_nn), so this is a plain compile-time constant rather than
// state living in Parameters.
inline constexpr int kNumOmpThreads = 5;

struct ImuData {
    int64_t time_ns;
    Eigen::Vector3d gyro;
    Eigen::Vector3d accel;
    Eigen::Matrix<double, 6, 24> H;
    Eigen::Matrix<double, 6, 1> imu_itp;
    ImuData(){}
    ImuData(const int64_t s, const Eigen::Vector3d& w, const Eigen::Vector3d& a)
      : time_ns(s), gyro(w), accel(a) {}

    ImuData(const ImuData& other) : time_ns(other.time_ns), gyro(other.gyro), accel(other.accel),
    H(other.H), imu_itp(other.imu_itp) {
    }          
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct WheelData {
    int64_t time_ns;
    Eigen::Vector3d vel;           // measured velocity in wheel/odometry frame O
    Eigen::Vector3d vel_itp;       // predicted velocity in frame O, filled by prepWheel
    Eigen::Matrix<double, 3, 24> H;
    WheelData(){}
    WheelData(const int64_t s, const Eigen::Vector3d& v) : time_ns(s), vel(v) {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct PoseData {
    int64_t time_ns;
    Eigen::Quaterniond orient;
    Eigen::Vector3d pos;
    PoseData(){}
    PoseData(int64_t s, Eigen::Quaterniond& q, Eigen::Vector3d& t) : time_ns(s), orient(q), pos(t) {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

namespace ouster_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    }EIGEN_ALIGN16;
}  

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
)

namespace hesai_ros {
  struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

namespace livox_mid360 {
    struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      uint8_t tag;
      uint8_t line;
      double timestamp;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  }
  POINT_CLOUD_REGISTER_POINT_STRUCT(livox_mid360::Point,
      (float, x, x)
      (float, y, y)
      (float, z, z)
      (float, intensity, intensity)
      (uint8_t, tag, tag)
      (uint8_t, line, line)
      (double, timestamp, timestamp)
  )
  
class CommonUtils
{
public:
    static constexpr double kGravity = 9.81;

    template <typename T>
    static T readParam(rclcpp::Node::SharedPtr &n, std::string name)
    {
        T ans;
        if (!n->has_parameter(name)) {
            n->declare_parameter<T>(name);
        }        
        if (!n->get_parameter(name, ans)) {
            RCLCPP_FATAL_STREAM(n->get_logger(), "Failed to load " << name);
            exit(1);
        }
        return ans;
    }

    template <typename T>
    static T readParam(rclcpp::Node::SharedPtr &n, std::string name, const T& alternative)
    {
        T ans;
        if (!n->has_parameter(name)) {
            n->declare_parameter<T>(name, alternative);
        }
        n->get_parameter_or(name, ans, alternative);
        return ans;
    }
    
    static Eigen::Vector3d readVector3d(rclcpp::Node::SharedPtr &n, const std::string& name)
    {
        std::vector<double> v = CommonUtils::readParam<std::vector<double>>(n, name);
        return Eigen::Vector3d(v[0], v[1], v[2]);
    }          

    static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / M_PI * 180.0;
    }      

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t y = ypr(0) / 180.0 * M_PI;
        Scalar_t p = ypr(1) / 180.0 * M_PI;
        Scalar_t r = ypr(2) / 180.0 * M_PI;

        Eigen::Matrix<Scalar_t, 3, 3> Rz;
        Rz << cos(y), -sin(y), 0,
                sin(y), cos(y), 0,
                0, 0, 1;

        Eigen::Matrix<Scalar_t, 3, 3> Ry;
        Ry << cos(p), 0., sin(p),
                0., 1., 0.,
                -sin(p), 0., cos(p);

        Eigen::Matrix<Scalar_t, 3, 3> Rx;
        Rx << 1., 0., 0.,
                0., cos(r), -sin(r),
                0., sin(r), cos(r);

        return Rz * Ry * Rx;
    }

    static Eigen::Matrix3d g2R(const Eigen::Vector3d &g)
    {
        Eigen::Matrix3d R0;
        Eigen::Vector3d ng1 = g.normalized();
        Eigen::Vector3d ng2{0, 0, 1.0};
        R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
        double yaw = CommonUtils::R2ypr(R0).x();
        R0 = CommonUtils::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
        return R0;
    }    

    static int64_t ms2ns(const float t_ms) {
        return (t_ms * float(1e6));
    }    

    static bool time_list(pcl::PointXYZINormal &x, pcl::PointXYZINormal &y) {return (x.intensity < y.intensity);};

    static geometry_msgs::msg::PoseStamped pose2msg(const int64_t t, const Eigen::Vector3d& pos,
                                              const Eigen::Quaterniond& orient)
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = rclcpp::Time(t);
        msg.pose.position.x = pos.x();
        msg.pose.position.y = pos.y();
        msg.pose.position.z = pos.z();
        msg.pose.orientation.w = orient.w();
        msg.pose.orientation.x = orient.x();
        msg.pose.orientation.y = orient.y();
        msg.pose.orientation.z = orient.z();
        return msg;
    }    

    static geometry_msgs::msg::Point32 getPointMsg(const Eigen::Vector3d& p)
    {
        geometry_msgs::msg::Point32 p_msg;
        p_msg.x = p.x();
        p_msg.y = p.y();
        p_msg.z = p.z();
        return p_msg;
    }    

    template<typename T>
    static bool esti_plane(Eigen::Matrix<T, 4, 1> &pca_result, const Eigen::aligned_vector<pcl::PointXYZINormal> &point, const T &threshold)
    {
        Eigen::Matrix<T, 5, 3> A;
        Eigen::Matrix<T, 5, 1> b;
        A.setZero();
        b.setOnes();
        b *= -1.0f;
        for (int j = 0; j < 5; j++)
        {
            A(j,0) = point[j].x;
            A(j,1) = point[j].y;
            A(j,2) = point[j].z;
        }
        Eigen::Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);
        T n = normvec.norm();
        pca_result(0) = normvec(0) / n;
        pca_result(1) = normvec(1) / n;
        pca_result(2) = normvec(2) / n;
        pca_result(3) = 1.0 / n;
        for (int j = 0; j < 5; j++)
        {
            if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
            {
                return false;
            }
        }
        return true;
    }            
};

struct LidarConfig {
    std::string topic;
    std::string type;
    int scan_line;
    float blind;
    Eigen::Quaterniond q_lb;
    Eigen::Vector3d t_lb;
    Eigen::Quaterniond q_bl;
    Eigen::Vector3d t_bl;
    double w_pt;

    LidarConfig() = default;

    LidarConfig(rclcpp::Node::SharedPtr& nh, const std::string& prefix) {
        std::cout << "Creating LidarConfig with prefix: \"" << prefix << "\"" << std::endl;
        topic = CommonUtils::readParam<std::string>(nh, prefix + "topic_lidar");
        type = CommonUtils::readParam<std::string>(nh, prefix + "lidar_type");
        scan_line = CommonUtils::readParam<int>(nh, prefix + "scan_line");
        blind = CommonUtils::readParam<float>(nh, prefix + "blind", 0.5f);
        std::vector<double> q_lb_v = CommonUtils::readParam<std::vector<double>>(nh, prefix + "q_lb");
        q_lb = Eigen::Quaterniond(q_lb_v.at(0), q_lb_v.at(1), q_lb_v.at(2), q_lb_v.at(3));
        t_lb = CommonUtils::readVector3d(nh, prefix + "t_lb");
        q_bl = q_lb.inverse();
        t_bl = q_lb.inverse() * (- t_lb);
        w_pt = CommonUtils::readParam<double>(nh, prefix + "w_pt");
    }
};

struct WheelConfig {
    bool enable = false;
    std::string topic;
    std::string topic_type;
    bool use_only_vx = true;
    Eigen::Vector3d p_bo = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_bo = Eigen::Matrix3d::Identity();
    double std_vx = 0.05;
    double std_vy = 0.02;
    double std_vz = 0.02;
    double max_allowed_residual_vx = 0.5;
    double adaptive_covariance_multiplier = 100.0;

    WheelConfig() = default;

    WheelConfig(rclcpp::Node::SharedPtr& nh) {
        enable = CommonUtils::readParam<bool>(nh, "wheel_odometry.enable", false);
        if (!enable) {
            return;
        }
        topic = CommonUtils::readParam<std::string>(nh, "wheel_odometry.topic_name");
        topic_type = CommonUtils::readParam<std::string>(nh, "wheel_odometry.topic_type");
        use_only_vx = CommonUtils::readParam<bool>(nh, "wheel_odometry.use_only_vx", true);
        std::vector<double> q_wb_v = CommonUtils::readParam<std::vector<double>>(nh, "wheel_odometry.q_wb");
        Eigen::Quaterniond q_wb(q_wb_v.at(0), q_wb_v.at(1), q_wb_v.at(2), q_wb_v.at(3));
        Eigen::Vector3d t_wb = CommonUtils::readVector3d(nh, "wheel_odometry.t_wb");
        R_bo = q_wb.inverse().toRotationMatrix();
        p_bo = q_wb.inverse() * (- t_wb);
        std_vx = CommonUtils::readParam<double>(nh, "wheel_odometry.std_vx");
        std_vy = CommonUtils::readParam<double>(nh, "wheel_odometry.std_vy");
        std_vz = CommonUtils::readParam<double>(nh, "wheel_odometry.std_vz");
        max_allowed_residual_vx = CommonUtils::readParam<double>(nh, "wheel_odometry.max_allowed_residual_vx");
        adaptive_covariance_multiplier = CommonUtils::readParam<double>(nh, "wheel_odometry.adaptive_covariance_multiplier");
    }
};

// Opt-in startup relocalization against a prior map (see Relocalization.h). When disabled
// (the default), RESPLE initializes as it always has, from scratch.
struct RelocConfig {
    bool enable = false;
    std::string map_path;
    bool initial_guess = false;
    Eigen::Vector3d t0 = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
    int icp_max_iterations = 50;
    double icp_max_corr_dist = 1.0;
    double icp_fitness_threshold = 0.5;

    RelocConfig() = default;

    RelocConfig(rclcpp::Node::SharedPtr& nh) {
        enable = CommonUtils::readParam<bool>(nh, "relocalization.enable", false);
        if (!enable) {
            return;
        }
        map_path = CommonUtils::readParam<std::string>(nh, "relocalization.map_path");
        initial_guess = CommonUtils::readParam<bool>(nh, "relocalization.initial_guess", false);
        if (initial_guess) {
            t0 = CommonUtils::readVector3d(nh, "relocalization.t0");
            std::vector<double> q0_v = CommonUtils::readParam<std::vector<double>>(nh, "relocalization.q0");
            q0 = Eigen::Quaterniond(q0_v.at(0), q0_v.at(1), q0_v.at(2), q0_v.at(3));
        }
        icp_max_iterations = CommonUtils::readParam<int>(nh, "relocalization.icp_max_iterations", 50);
        icp_max_corr_dist = CommonUtils::readParam<double>(nh, "relocalization.icp_max_corr_dist", 1.0);
        icp_fitness_threshold = CommonUtils::readParam<double>(nh, "relocalization.icp_fitness_threshold", 0.5);
    }
};

struct Parameters {
    Eigen::Vector3d cov_acc;
    Eigen::Vector3d cov_gyro;
    Eigen::Vector3d gravity;
    double nn_thresh;
    double coeff_cov;
    int num_nn;

    Parameters() {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointData {
    int64_t time_ns;
    pcl::PointXYZINormal pt;
    Eigen::Vector3d pt_b;
    pcl::PointXYZINormal pt_w;
    Eigen::Vector3d normvec;
    bool if_valid;
    double dist;
    Eigen::aligned_vector<pcl::PointXYZINormal> nearest_points;
    double zp = 0;
    Eigen::Matrix<double, 1, 24> H = Eigen::Matrix<double, 1, 24>::Zero();
    Eigen::Quaterniond q_bl;
    Eigen::Vector3d t_bl;    
    double var_pt;

    PointData() {};
    PointData(const pcl::PointXYZINormal& pt_in, int64_t fr_start_time, const Eigen::Quaterniond& q_bl_in,
        const Eigen::Vector3d& t_bl_in, double w_pt) : pt(pt_in), pt_b(Eigen::Vector3d(pt_in.x, pt_in.y, pt_in.z)), 
        q_bl(q_bl_in), t_bl(t_bl_in) {
        time_ns = fr_start_time + CommonUtils::ms2ns(pt_in.intensity);
        if_valid = false;
        dist = 0;
        var_pt = w_pt;
    }

    PointData(const PointData& other) = default;
    PointData& operator=(const PointData& other) = default;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


