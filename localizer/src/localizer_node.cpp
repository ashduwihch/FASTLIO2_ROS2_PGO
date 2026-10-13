// ROS 2 port of HViktorTsoi/FAST_LIO_LOCALIZATION's global localization and
// transform-fusion pipeline. Licensed under GPL-2.0-only.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <open3d/Open3D.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

#include "interface/srv/is_valid.hpp"
#include "interface/srv/relocalize.hpp"

using PointT = pcl::PointXYZI;
using PclCloud = pcl::PointCloud<PointT>;
using O3dCloud = open3d::geometry::PointCloud;

namespace
{
Eigen::Matrix4d poseToMatrix(const geometry_msgs::msg::Pose &pose)
{
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                         pose.orientation.y, pose.orientation.z);
    q = q.norm() < 1e-9 ? Eigen::Quaterniond::Identity() : q.normalized();
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
    t.block<3, 3>(0, 0) = q.toRotationMatrix();
    t.block<3, 1>(0, 3) = Eigen::Vector3d(
        pose.position.x, pose.position.y, pose.position.z);
    return t;
}

geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4d &t)
{
    geometry_msgs::msg::Pose pose;
    Eigen::Quaterniond q(t.block<3, 3>(0, 0));
    q.normalize();
    pose.position.x = t(0, 3);
    pose.position.y = t(1, 3);
    pose.position.z = t(2, 3);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}

std::shared_ptr<O3dCloud> pclToOpen3d(const PclCloud &cloud)
{
    auto result = std::make_shared<O3dCloud>();
    result->points_.reserve(cloud.size());
    for (const auto &p : cloud.points)
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
            result->points_.emplace_back(p.x, p.y, p.z);
    return result;
}

PclCloud::Ptr open3dToPcl(const O3dCloud &cloud, std::size_t stride)
{
    auto result = std::make_shared<PclCloud>();
    stride = std::max<std::size_t>(stride, 1);
    result->reserve((cloud.points_.size() + stride - 1) / stride);
    for (std::size_t i = 0; i < cloud.points_.size(); i += stride)
    {
        PointT p;
        p.x = cloud.points_[i].x();
        p.y = cloud.points_[i].y();
        p.z = cloud.points_[i].z();
        p.intensity = 0.0F;
        result->push_back(p);
    }
    result->width = result->size();
    result->height = 1;
    result->is_dense = true;
    return result;
}
}  // namespace

struct Config
{
    std::string cloud_topic = "/fastlio2/world_cloud";
    std::string odom_topic = "/fastlio2/lio_odom";
    std::string map_frame = "map";
    std::string odom_frame = "lidar";
    std::string body_frame = "body";
    std::string map_path;
    double map_voxel_size = 0.4;
    double scan_voxel_size = 0.1;
    double localization_frequency = 0.5;
    double publish_frequency = 50.0;
    double localization_threshold = 0.95;
    double fov = 2.0 * M_PI;
    double fov_far = 30.0;
    int max_iteration = 20;
    int submap_publish_stride = 10;
    bool override_initial_z = true;
    double initial_z = 0.5;
};

class LocalizerNode : public rclcpp::Node
{
public:
    LocalizerNode() : Node("localizer_node")
    {
        RCLCPP_INFO(get_logger(), "FAST_LIO_LOCALIZATION ROS 2 port started");
        loadParameters();
        sensor_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        localization_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        fusion_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sensor_options;
        sensor_options.callback_group = sensor_group_;
        rclcpp::SubscriptionOptions localization_options;
        localization_options.callback_group = localization_group_;
        const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(5);
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            config_.cloud_topic, sensor_qos,
            std::bind(&LocalizerNode::cloudCallback, this, std::placeholders::_1),
            sensor_options);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            config_.odom_topic, sensor_qos,
            std::bind(&LocalizerNode::odomCallback, this, std::placeholders::_1),
            sensor_options);
        initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&LocalizerNode::initialPoseCallback, this, std::placeholders::_1),
            localization_options);

        map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/map", rclcpp::QoS(1).reliable().transient_local());
        scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cur_scan_in_map", 1);
        submap_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/submap", 1);
        map_to_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/map_to_odom", 1);
        localization_pub_ = create_publisher<nav_msgs::msg::Odometry>("/localization", 1);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        relocalize_service_ = create_service<interface::srv::Relocalize>(
            "relocalize", std::bind(&LocalizerNode::relocalizeCallback, this,
                                     std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, localization_group_);
        check_service_ = create_service<interface::srv::IsValid>(
            "relocalize_check", std::bind(&LocalizerNode::checkCallback, this,
                                           std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, sensor_group_);

        if (!loadMap(config_.map_path))
            RCLCPP_ERROR(get_logger(), "Prior map unavailable; localization is disabled");

        localization_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(1.0 / std::max(config_.localization_frequency, 1e-3))),
            std::bind(&LocalizerNode::periodicLocalization, this), localization_group_);
        fusion_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(1.0 / std::max(config_.publish_frequency, 1e-3))),
            std::bind(&LocalizerNode::publishFusion, this), fusion_group_);
        RCLCPP_WARN(get_logger(), "Waiting for RViz 2D Pose Estimate (/initialpose)...");
    }

private:
    void loadParameters()
    {
        declare_parameter("config_path", "");
        const std::string path = get_parameter("config_path").as_string();
        const YAML::Node y = YAML::LoadFile(path);
        RCLCPP_INFO(get_logger(), "LOAD FROM YAML CONFIG PATH: %s", path.c_str());
        auto s = [&y](const char *k, const std::string &v) { return y[k] ? y[k].as<std::string>() : v; };
        auto d = [&y](const char *k, double v) { return y[k] ? y[k].as<double>() : v; };
        auto i = [&y](const char *k, int v) { return y[k] ? y[k].as<int>() : v; };
        auto b = [&y](const char *k, bool v) { return y[k] ? y[k].as<bool>() : v; };
        config_.cloud_topic = s("cloud_topic", config_.cloud_topic);
        config_.odom_topic = s("odom_topic", config_.odom_topic);
        config_.map_frame = s("map_frame", config_.map_frame);
        config_.odom_frame = s("odom_frame", config_.odom_frame);
        config_.body_frame = s("body_frame", config_.body_frame);
        config_.map_path = s("map_path", config_.map_path);
        config_.map_voxel_size = d("map_voxel_size", config_.map_voxel_size);
        config_.scan_voxel_size = d("scan_voxel_size", config_.scan_voxel_size);
        config_.localization_frequency = d("localization_frequency", config_.localization_frequency);
        config_.publish_frequency = d("publish_frequency", config_.publish_frequency);
        config_.localization_threshold = d("localization_threshold", config_.localization_threshold);
        config_.fov = d("fov", config_.fov);
        config_.fov_far = d("fov_far", config_.fov_far);
        config_.max_iteration = i("max_iteration", config_.max_iteration);
        config_.submap_publish_stride = i("submap_publish_stride", config_.submap_publish_stride);
        config_.override_initial_z = b("override_initial_z", config_.override_initial_z);
        config_.initial_z = d("initial_z", config_.initial_z);
    }

    bool loadMap(const std::string &path)
    {
        if (path.empty() || !std::filesystem::exists(path))
        {
            RCLCPP_ERROR(get_logger(), "PCD map not found: %s", path.c_str());
            return false;
        }
        PclCloud pcl_map;
        if (pcl::io::loadPCDFile<PointT>(path, pcl_map) < 0)
            return false;
        auto map = pclToOpen3d(pcl_map)->VoxelDownSample(config_.map_voxel_size);
        if (map->IsEmpty())
            return false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            global_map_ = map;
            config_.map_path = path;
            map_loaded_ = true;
        }
        publishCloud(map_pub_, *map, config_.map_frame, now(), 1);
        RCLCPP_INFO(get_logger(), "Global map received: %zu points (%s)", map->points_.size(), path.c_str());
        return true;
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        PclCloud pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);
        auto scan = pclToOpen3d(pcl_cloud);
        if (scan->IsEmpty())
            return;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            current_scan_ = scan;
        }
        auto display = *msg;
        display.header.frame_id = config_.odom_frame;
        scan_pub_->publish(display);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_odom_ = msg;
    }

    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (!msg->header.frame_id.empty() && msg->header.frame_id != config_.map_frame)
        {
            RCLCPP_WARN(get_logger(), "Initial pose frame [%s] is not [%s]",
                        msg->header.frame_id.c_str(), config_.map_frame.c_str());
            return;
        }
        Eigen::Matrix4d initial = poseToMatrix(msg->pose.pose);
        // RViz 2D Pose Estimate sends z=0; keep a configurable UAV height for
        // that case, but preserve an explicitly supplied 6-DoF z value.
        if (config_.override_initial_z && std::abs(initial(2, 3)) < 1e-6)
            initial(2, 3) = config_.initial_z;
        RCLCPP_INFO(get_logger(), "Initial pose: x=%.3f y=%.3f z=%.3f",
                    initial(0, 3), initial(1, 3), initial(2, 3));
        runLocalization(initial, true);
    }

    void relocalizeCallback(const std::shared_ptr<interface::srv::Relocalize::Request> req,
                            std::shared_ptr<interface::srv::Relocalize::Response> res)
    {
        if (!req->pcd_path.empty() && req->pcd_path != config_.map_path && !loadMap(req->pcd_path))
        {
            res->success = false; res->message = "load map failed"; return;
        }
        const Eigen::AngleAxisd yaw(req->yaw, Eigen::Vector3d::UnitZ());
        const Eigen::AngleAxisd pitch(req->pitch, Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd roll(req->roll, Eigen::Vector3d::UnitX());
        Eigen::Matrix4d initial = Eigen::Matrix4d::Identity();
        initial.block<3, 3>(0, 0) = (yaw * pitch * roll).toRotationMatrix();
        initial.block<3, 1>(0, 3) = Eigen::Vector3d(req->x, req->y, req->z);
        res->success = runLocalization(initial, true);
        res->message = res->success ? "relocalization succeeded" : "relocalization failed";
    }

    void checkCallback(const std::shared_ptr<interface::srv::IsValid::Request> req,
                       std::shared_ptr<interface::srv::IsValid::Response> res)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        res->valid = req->code == 1 || initialized_;
    }

    void periodicLocalization()
    {
        Eigen::Matrix4d estimate;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!initialized_ || matching_) return;
            estimate = map_to_odom_;
        }
        runLocalization(estimate, false);
    }

    bool runLocalization(const Eigen::Matrix4d &estimate, bool initialization)
    {
        std::shared_ptr<O3dCloud> scan, map;
        nav_msgs::msg::Odometry::SharedPtr odom;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (matching_) { RCLCPP_WARN(get_logger(), "Localization already running"); return false; }
            if (!map_loaded_ || !global_map_) { RCLCPP_WARN(get_logger(), "Waiting for global map"); return false; }
            if (!current_scan_ || !current_odom_) { RCLCPP_WARN(get_logger(), "First scan or odometry not received"); return false; }
            matching_ = true;
            scan = std::make_shared<O3dCloud>(*current_scan_);
            map = global_map_;
            odom = std::make_shared<nav_msgs::msg::Odometry>(*current_odom_);
        }

        RCLCPP_INFO(get_logger(), "Global localization by scan-to-map matching...");
        const auto start = std::chrono::steady_clock::now();
        auto submap = cropMap(*map, estimate, *odom);
        publishCloud(submap_pub_, *submap, config_.map_frame, rclcpp::Time(odom->header.stamp),
                     config_.submap_publish_stride);
        const bool valid = !scan->IsEmpty() && !submap->IsEmpty();
        open3d::pipelines::registration::RegistrationResult coarse, fine;
        if (valid)
        {
            coarse = registrationAtScale(*scan, *submap, estimate, 5.0);
            fine = registrationAtScale(*scan, *submap, coarse.transformation_, 1.0);
        }
        const double seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
        const double fitness = valid ? fine.fitness_ : 0.0;
        const double rmse = valid ? fine.inlier_rmse_ : 0.0;
        Eigen::Matrix4d transform = estimate;
        if (valid)
            transform = fine.transformation_;
        std::ostringstream matrix;
        matrix << std::fixed << std::setprecision(4) << transform;
        RCLCPP_INFO(get_logger(), "Time: %.3f s", seconds);
        RCLCPP_INFO(get_logger(), "coarse fitness: %.6f", valid ? coarse.fitness_ : 0.0);
        RCLCPP_INFO(get_logger(), "fitness score: %.6f, inlier RMSE: %.6f", fitness, rmse);
        RCLCPP_INFO(get_logger(), "estimated map_to_odom:\n%s", matrix.str().c_str());

        const bool success = valid && fitness > config_.localization_threshold;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (success) { map_to_odom_ = transform; initialized_ = true; }
            matching_ = false;
        }
        if (success)
        {
            publishMapToOdom(transform, odom->header.stamp);
            if (initialization) RCLCPP_INFO(get_logger(), "Initialize successfully!");
        }
        else
            RCLCPP_WARN(get_logger(), "Not match! fitness %.6f <= threshold %.6f",
                        fitness, config_.localization_threshold);
        return success;
    }

    std::shared_ptr<O3dCloud> cropMap(const O3dCloud &map, const Eigen::Matrix4d &estimate,
                                      const nav_msgs::msg::Odometry &odom) const
    {
        const Eigen::Matrix4d body_from_map = (estimate * poseToMatrix(odom.pose.pose)).inverse();
        auto result = std::make_shared<O3dCloud>();
        result->points_.reserve(map.points_.size());
        for (const auto &p : map.points_)
        {
            const Eigen::Vector3d body = (body_from_map * Eigen::Vector4d(p.x(), p.y(), p.z(), 1.0)).head<3>();
            const double angle = std::abs(std::atan2(body.y(), body.x()));
            const bool inside = config_.fov > M_PI
                ? body.x() < config_.fov_far && angle < config_.fov / 2.0
                : body.x() > 0.0 && body.x() < config_.fov_far && angle < config_.fov / 2.0;
            if (inside) result->points_.push_back(p);
        }
        return result;
    }

    open3d::pipelines::registration::RegistrationResult registrationAtScale(
        const O3dCloud &scan, const O3dCloud &map, const Eigen::Matrix4d &initial, double scale) const
    {
        const auto scan_down = scan.VoxelDownSample(config_.scan_voxel_size * scale);
        const auto map_down = map.VoxelDownSample(config_.map_voxel_size * scale);
        const open3d::pipelines::registration::TransformationEstimationPointToPoint estimation(false);
        const open3d::pipelines::registration::ICPConvergenceCriteria criteria(1e-6, 1e-6, config_.max_iteration);
        return open3d::pipelines::registration::RegistrationICP(
            *scan_down, *map_down, scale, initial, estimation, criteria);
    }

    void publishFusion()
    {
        Eigen::Matrix4d map_to_odom;
        nav_msgs::msg::Odometry::SharedPtr odom;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            map_to_odom = map_to_odom_;
            if (current_odom_) odom = std::make_shared<nav_msgs::msg::Odometry>(*current_odom_);
        }
        // Do not mix the host clock with Gazebo sensor time.  Publishing a TF
        // stamped with now() before the first odometry message poisons RViz's
        // TF cache: all subsequent simulation-time transforms look ancient.
        if (!odom)
            return;

        geometry_msgs::msg::TransformStamped tf;
        // FAST-LIO/Gazebo messages carry simulation sensor time.  Stamp the TF
        // from the same clock so RViz does not reject scans as TF_OLD_DATA.
        tf.header.stamp = odom->header.stamp;
        tf.header.frame_id = config_.map_frame;
        tf.child_frame_id = config_.odom_frame;
        const auto pose = matrixToPose(map_to_odom);
        tf.transform.translation.x = pose.position.x;
        tf.transform.translation.y = pose.position.y;
        tf.transform.translation.z = pose.position.z;
        tf.transform.rotation = pose.orientation;
        tf_broadcaster_->sendTransform(tf);

        nav_msgs::msg::Odometry localization;
        localization.header.stamp = odom->header.stamp;
        localization.header.frame_id = config_.map_frame;
        localization.child_frame_id = config_.body_frame;
        localization.pose.pose = matrixToPose(map_to_odom * poseToMatrix(odom->pose.pose));
        localization.pose.covariance = odom->pose.covariance;
        localization.twist = odom->twist;
        localization_pub_->publish(localization);
    }

    void publishMapToOdom(const Eigen::Matrix4d &transform,
                          const builtin_interfaces::msg::Time &stamp)
    {
        nav_msgs::msg::Odometry msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = config_.map_frame;
        msg.child_frame_id = config_.odom_frame;
        msg.pose.pose = matrixToPose(transform);
        map_to_odom_pub_->publish(msg);
    }

    void publishCloud(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub,
                      const O3dCloud &cloud, const std::string &frame,
                      const rclcpp::Time &stamp, std::size_t stride)
    {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*open3dToPcl(cloud, stride), msg);
        msg.header.frame_id = frame;
        msg.header.stamp = stamp;
        pub->publish(msg);
    }

    Config config_;
    std::mutex mutex_;
    std::shared_ptr<O3dCloud> global_map_, current_scan_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    Eigen::Matrix4d map_to_odom_ = Eigen::Matrix4d::Identity();
    bool map_loaded_ = false, initialized_ = false, matching_ = false;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_, scan_pub_, submap_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_to_odom_pub_, localization_pub_;
    rclcpp::Service<interface::srv::Relocalize>::SharedPtr relocalize_service_;
    rclcpp::Service<interface::srv::IsValid>::SharedPtr check_service_;
    rclcpp::TimerBase::SharedPtr localization_timer_, fusion_timer_;
    rclcpp::CallbackGroup::SharedPtr sensor_group_, localization_group_, fusion_group_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizerNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
