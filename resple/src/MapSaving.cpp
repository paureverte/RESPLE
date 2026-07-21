#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "estimate_msgs/srv/save_map.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <mutex>

class MapSaving : public rclcpp::Node
{
public:
    MapSaving() : Node("MapSaving")
    {
        pcd_save_path = this->declare_parameter<std::string>("mapping.global_pcd_save_path", "/tmp/resple_global_map.pcd");

        sub_global_map = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "global_map", 200,
            std::bind(&MapSaving::globalMapCallback, this, std::placeholders::_1));

        srv_save_map = this->create_service<estimate_msgs::srv::SaveMap>(
            "save_global_map",
            std::bind(&MapSaving::savePCDCallback, this, std::placeholders::_1, std::placeholders::_2));

        accumulated_map.reset(new pcl::PointCloud<pcl::PointXYZI>());

        RCLCPP_INFO(this->get_logger(), "MapSaving node started, subscribing to 'global_map'.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_global_map;
    rclcpp::Service<estimate_msgs::srv::SaveMap>::SharedPtr srv_save_map;
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_map;
    std::mutex mtx_map;
    std::string pcd_save_path;

    void globalMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);
        std::lock_guard<std::mutex> lock(mtx_map);
        *accumulated_map += *cloud;
    }

    void savePCDCallback(const std::shared_ptr<estimate_msgs::srv::SaveMap::Request> request,
                         std::shared_ptr<estimate_msgs::srv::SaveMap::Response> response)
    {
        const std::string& save_path = request->path.empty() ? pcd_save_path : request->path;
        pcl::PointCloud<pcl::PointXYZI> map_copy;
        {
            std::lock_guard<std::mutex> lock(mtx_map);
            map_copy = *accumulated_map;
        }
        int ret = pcl::io::savePCDFileBinary(save_path, map_copy);
        response->success = (ret == 0);
        response->message = response->success
            ? "Saved map to " + save_path + " (" + std::to_string(map_copy.size()) + " points)"
            : "Failed to save map to " + save_path;
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapSaving>());
    rclcpp::shutdown();
    return 0;
}
