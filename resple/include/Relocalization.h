#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include "utils/common_utils.h"

// Startup-only relocalization against a prior map: load a previously-saved point
// cloud (see README's Relocalization section — use /save_global_map's output, not
// /save_map's bounded local map), then refine a coarse initial pose guess against
// it via ICP. This is local refinement only, not place recognition: a guess that's
// far from the true pose will simply fail to converge, not silently find it.
class Relocalization
{
public:
    // Loads generically as XYZI rather than directly as pcl::PointXYZINormal.
    // Both /save_map's (PointXYZINormal) and /save_global_map's (PointXYZI) output
    // are guaranteed to have x/y/z/intensity, but /save_global_map's — the source
    // this feature's README recommends — has no normal_x/y/z/curvature fields at
    // all, and loadPCDFile<PointXYZINormal> loading a file missing fields it wants
    // has been observed to crash (not just warn) in PCL's field-mapping. normal_x/
    // y/z/curvature are set to 0 below; nothing downstream (ICP, ikd-Tree, or the
    // prior_map RViz display, which uses a flat color) reads them.
    static bool loadPriorMap(const std::string& path, pcl::PointCloud<pcl::PointXYZINormal>::Ptr& out_cloud)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>());
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(path, *raw) != 0 || raw->empty()) {
            return false;
        }
        out_cloud.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
        out_cloud->resize(raw->size());
        for (size_t i = 0; i < raw->size(); i++) {
            pcl::PointXYZINormal& po = out_cloud->points[i];
            const pcl::PointXYZI& pi = raw->points[i];
            po.x = pi.x;
            po.y = pi.y;
            po.z = pi.z;
            po.intensity = pi.intensity;
            po.normal_x = po.normal_y = po.normal_z = 0.0f;
            po.curvature = 0.0f;
        }
        return true;
    }

    // Sensor -> body frame only (the static per-LiDAR extrinsic) — deliberately NOT
    // Association::pointBodyToWorld, since there is no meaningful world pose yet to
    // spline-interpolate against at this point in startup.
    static void pointSensorToBody(const pcl::PointXYZINormal& pi, pcl::PointXYZINormal& po,
                                   const Eigen::Vector3d& t_bl, const Eigen::Quaterniond& q_bl)
    {
        Eigen::Vector3d p_body = q_bl * Eigen::Vector3d(pi.x, pi.y, pi.z) + t_bl;
        po.x = p_body.x();
        po.y = p_body.y();
        po.z = p_body.z();
        po.curvature = pi.curvature;
    }

    // source_body_frame: live scan window transformed by pointSensorToBody only (body
    // frame at start_t_ns). target_map: the loaded prior map (world frame). t_guess/
    // q_guess and t_out/q_out are the body->world transform, matching initFilter's
    // (start_t_ns, t_init, q_init) semantics exactly. Returns false (t_out/q_out left
    // untouched) if ICP fails to converge or exceeds cfg.icp_fitness_threshold;
    // fitness_out is set either way, for logging.
    static bool refineInitialPose(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& source_body_frame,
                                   const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& target_map,
                                   const Eigen::Vector3d& t_guess, const Eigen::Quaterniond& q_guess,
                                   const RelocConfig& cfg,
                                   Eigen::Vector3d& t_out, Eigen::Quaterniond& q_out, double& fitness_out)
    {
        Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
        guess.topLeftCorner<3, 3>() = q_guess.cast<float>().toRotationMatrix();
        guess.topRightCorner<3, 1>() = t_guess.cast<float>();

        pcl::IterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal> icp;
        icp.setInputSource(source_body_frame);
        icp.setInputTarget(target_map);
        icp.setMaximumIterations(cfg.icp_max_iterations);
        icp.setMaxCorrespondenceDistance(cfg.icp_max_corr_dist);
        pcl::PointCloud<pcl::PointXYZINormal> aligned;
        icp.align(aligned, guess);

        fitness_out = icp.getFitnessScore();
        if (!icp.hasConverged() || fitness_out > cfg.icp_fitness_threshold) {
            return false;
        }
        Eigen::Matrix4f T = icp.getFinalTransformation();
        Eigen::Matrix3d R = T.topLeftCorner<3, 3>().cast<double>();
        t_out = T.topRightCorner<3, 1>().cast<double>();
        q_out = Eigen::Quaterniond(R);
        return true;
    }
};
