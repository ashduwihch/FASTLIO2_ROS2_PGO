#include <iostream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include "hba/hba.h"
#include <pcl/filters/voxel_grid.h>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <cmath>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "interface/srv/refine_map.hpp"
#include "interface/srv/save_poses.hpp"

using namespace std::chrono_literals;
void fromStr(const std::string &str, std::string &file_name, Pose &pose)
{
    std::stringstream ss(str);
    std::vector<std::string> tokens;
    std::string token;
    while (std::getline(ss, token, ' '))
    {
        tokens.push_back(token);
    }
    assert(tokens.size() == 8);
    file_name = tokens[0];
    pose.t = Eigen::Vector3d(std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]));
    pose.r = Eigen::Quaterniond(std::stod(tokens[4]), std::stod(tokens[5]), std::stod(tokens[6]), std::stod(tokens[7])).normalized().toRotationMatrix();
}

struct NodeConfig
{
    double scan_resolution = 0.1;
    double correction_display_scale = 10.0;
    int max_correction_arrows = 100;
    double correction_marker_size = 0.12;
};

class HBANode : public rclcpp::Node
{
public:
    HBANode() : Node("hba_node")
    {
        RCLCPP_INFO(this->get_logger(), "HBA node started");
        loadParameters();
        m_hba = std::make_shared<HBA>(m_hba_config);
        m_voxel_grid.setLeafSize(m_node_config.scan_resolution, m_node_config.scan_resolution, m_node_config.scan_resolution);
        m_refine_map_srv = this->create_service<interface::srv::RefineMap>(
            "refine_map",
            std::bind(&HBANode::refineMapCB, this, std::placeholders::_1, std::placeholders::_2));
        m_save_poses_srv = this->create_service<interface::srv::SavePoses>(
            "save_poses",
            std::bind(&HBANode::savePosesCB, this, std::placeholders::_1, std::placeholders::_2));
        m_timer = this->create_wall_timer(100ms, std::bind(&HBANode::mainCB, this));
        rclcpp::QoS map_qos(rclcpp::KeepLast(1));
        map_qos.reliable().transient_local();
        m_before_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_before", map_qos);
        m_after_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_after", map_qos);
        // Keep the original topic for compatibility with existing users.
        m_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_points", map_qos);
        m_visualization_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "optimization_visualization", map_qos);
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_node_config.scan_resolution = config["scan_resolution"].as<double>();
        if (config["correction_display_scale"])
            m_node_config.correction_display_scale = config["correction_display_scale"].as<double>();
        if (config["max_correction_arrows"])
            m_node_config.max_correction_arrows = config["max_correction_arrows"].as<int>();
        if (config["correction_marker_size"])
            m_node_config.correction_marker_size = config["correction_marker_size"].as<double>();
        m_hba_config.total_layer_num = config["total_layer_num"].as<int>();
        m_hba_config.thread_num = config["thread_num"].as<int>();
        m_hba_config.max_iter = config["max_iter"].as<int>();
        m_hba_config.down_sample = config["down_sample"].as<double>();
        m_hba_config.voxel_size = config["voxel_size"].as<double>();
        m_hba_config.eigen_ratio = config["eigen_ratio"].as<double>();
        m_hba_config.reject_ratio = config["reject_ratio"].as<double>();
    }

    void refineMapCB(const std::shared_ptr<interface::srv::RefineMap::Request> request, std::shared_ptr<interface::srv::RefineMap::Response> response)
    {
        std::lock_guard<std::mutex> lock(m_service_mutex);
        if (!std::filesystem::exists(request->maps_path))
        {
            response->success = false;
            response->message = "maps_path not exists";
            return;
        }

        std::filesystem::path p_dir(request->maps_path);
        std::filesystem::path pcd_dir = p_dir / "patches";
        if (!std::filesystem::exists(pcd_dir))
        {
            response->success = false;
            response->message = pcd_dir.string() + " not exists";
            return;
        }

        std::filesystem::path txt_file = p_dir / "poses.txt";

        if (!std::filesystem::exists(txt_file))
        {
            response->success = false;
            response->message = txt_file.string() + " not exists";
            return;
        }

        std::ifstream ifs(txt_file);
        // A new request represents a new optimization job. Clear the previous
        // scans so calling the service twice does not append duplicate patches.
        m_hba = std::make_shared<HBA>(m_hba_config);
        std::string line;
        std::string file_name;
        Pose pose;
        pcl::PCDReader reader;
        while (std::getline(ifs, line))
        {

            fromStr(line, file_name, pose);
            std::filesystem::path pcd_file = p_dir / "patches" / file_name;
            if (!std::filesystem::exists(pcd_file))
            {
                std::cerr << "pcd file not found" << std::endl;
                continue;
            }
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            reader.read(pcd_file, *cloud);
            m_voxel_grid.setInputCloud(cloud);
            m_voxel_grid.filter(*cloud);
            m_hba->insert(cloud, pose);
        }
        RCLCPP_INFO(this->get_logger(), "LOAD POSE %lu;", m_hba->poses().size());
        m_initial_poses = m_hba->poses();
        m_maps_path = p_dir;
        response->success = true;
        response->message = "load poses success!";
        m_do_optimize = true;
        return;
    }

    void savePosesCB(const std::shared_ptr<interface::srv::SavePoses::Request> request, std::shared_ptr<interface::srv::SavePoses::Response> response)
    {
        std::lock_guard<std::mutex> lock(m_service_mutex);
        std::filesystem::path file_path(request->file_path);
        std::filesystem::path par_path = file_path.parent_path();
        if (!std::filesystem::exists(par_path))
        {
            response->success = false;
            response->message = "parent path not exists";
            return;
        }
        if (m_hba->poses().size() < 1)
        {
            response->success = false;
            response->message = "poses is empty";
            return;
        }

        if (std::filesystem::exists(file_path))
        {
            std::filesystem::remove(file_path);
        }

        m_hba->writePoses(file_path);
        response->success = true;
        response->message = "save poses success!";
        return;
    }
    void mainCB()
    {
        {
            std::lock_guard<std::mutex> lock(m_service_mutex);
            if (!m_do_optimize)
                return;
            RCLCPP_WARN(this->get_logger(), "START OPTIMIZE");
            auto before_cloud = getFilteredMap();
            publishCloud(m_before_cloud_pub, before_cloud);
            publishCloud(m_cloud_pub, before_cloud);
            saveMap(before_cloud, m_maps_path / "map_before_hba.pcd");
            std::filesystem::remove(m_maps_path / "map_after_hba.pcd");
            std::filesystem::remove(m_maps_path / "hba_poses.txt");
            try
            {
                RCLCPP_INFO(this->get_logger(), "======OFFICIAL HBA START======");
                m_hba->optimize();
                std::string validation_message;
                if (!validateOptimization(validation_message))
                {
                    m_hba->poses() = m_initial_poses;
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "HBA result rejected and original poses restored: %s",
                        validation_message.c_str());
                    m_do_optimize = false;
                    return;
                }
                auto iteration_cloud = getFilteredMap();
                publishCloud(m_after_cloud_pub, iteration_cloud);
                publishCloud(m_cloud_pub, iteration_cloud);
                RCLCPP_INFO(this->get_logger(), "======OFFICIAL HBA END========");
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "HBA optimization failed: %s", e.what());
                m_do_optimize = false;
                return;
            }

            auto after_cloud = getFilteredMap();
            publishCloud(m_after_cloud_pub, after_cloud);
            publishCloud(m_cloud_pub, after_cloud);
            saveMap(after_cloud, m_maps_path / "map_after_hba.pcd");
            m_hba->writePoses((m_maps_path / "hba_poses.txt").string());
            publishOptimizationVisualization();
            printCorrectionStatistics();
            m_do_optimize = false;
            RCLCPP_WARN(this->get_logger(), "END OPTIMIZE");
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr getFilteredMap()
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = m_hba->getMapPoints();
        m_voxel_grid.setInputCloud(cloud);
        m_voxel_grid.filter(*cloud);
        return cloud;
    }

    void publishCloud(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
    {
        if (!publisher || cloud->empty())
            return;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = "map";
        cloud_msg.header.stamp = this->now();
        publisher->publish(cloud_msg);
    }

    void saveMap(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
        const std::filesystem::path &file_path)
    {
        if (pcl::io::savePCDFileBinary(file_path.string(), *cloud) == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Saved %s (%lu points)",
                        file_path.c_str(), cloud->size());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save %s", file_path.c_str());
        }
    }

    void printCorrectionStatistics()
    {
        const auto &optimized_poses = m_hba->poses();
        if (m_initial_poses.size() != optimized_poses.size() || optimized_poses.empty())
            return;

        double translation_sum = 0.0;
        double translation_max = 0.0;
        double rotation_sum_deg = 0.0;
        double rotation_max_deg = 0.0;
        size_t max_translation_index = 0;
        size_t max_rotation_index = 0;

        for (size_t i = 0; i < optimized_poses.size(); ++i)
        {
            const double translation = (optimized_poses[i].t - m_initial_poses[i].t).norm();
            const Eigen::Matrix3d delta_rotation =
                m_initial_poses[i].r.transpose() * optimized_poses[i].r;
            const double rotation_deg =
                std::abs(Eigen::AngleAxisd(delta_rotation).angle()) * 180.0 / M_PI;

            translation_sum += translation;
            rotation_sum_deg += rotation_deg;
            if (translation > translation_max)
            {
                translation_max = translation;
                max_translation_index = i;
            }
            if (rotation_deg > rotation_max_deg)
            {
                rotation_max_deg = rotation_deg;
                max_rotation_index = i;
            }
        }

        const double count = static_cast<double>(optimized_poses.size());
        RCLCPP_WARN(
            this->get_logger(),
            "HBA CORRECTION: mean translation=%.4f m, max translation=%.4f m (pose %lu), "
            "mean rotation=%.3f deg, max rotation=%.3f deg (pose %lu)",
            translation_sum / count, translation_max, max_translation_index,
            rotation_sum_deg / count, rotation_max_deg, max_rotation_index);
        RCLCPP_INFO(this->get_logger(), "HBA outputs: %s, %s, %s",
                    (m_maps_path / "map_before_hba.pcd").c_str(),
                    (m_maps_path / "map_after_hba.pcd").c_str(),
                    (m_maps_path / "hba_poses.txt").c_str());
    }

    std_msgs::msg::ColorRGBA correctionColor(double ratio) const
    {
        ratio = std::clamp(ratio, 0.0, 1.0);
        std_msgs::msg::ColorRGBA color;
        color.a = 1.0F;
        // Small corrections are cyan/green, medium corrections yellow,
        // and the largest corrections red.
        if (ratio < 0.5)
        {
            color.r = static_cast<float>(ratio * 2.0);
            color.g = 1.0F;
            color.b = static_cast<float>(1.0 - ratio * 2.0);
        }
        else
        {
            color.r = 1.0F;
            color.g = static_cast<float>(2.0 * (1.0 - ratio));
            color.b = 0.0F;
        }
        return color;
    }

    void publishOptimizationVisualization()
    {
        const auto &optimized = m_hba->poses();
        if (!m_visualization_pub || optimized.size() != m_initial_poses.size() || optimized.empty())
            return;

        const auto stamp = this->now();

        // Publish DELETEALL separately. RViz may apply DELETEALL after the
        // other markers when it is mixed into the same MarkerArray.
        visualization_msgs::msg::MarkerArray clear_markers;
        visualization_msgs::msg::Marker clear;
        clear.header.frame_id = "map";
        clear.header.stamp = stamp;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        clear_markers.markers.push_back(clear);
        m_visualization_pub->publish(clear_markers);

        visualization_msgs::msg::MarkerArray markers;

        auto makeTrajectory = [&](const std::string &name, int id,
                                  float r, float g, float b)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = stamp;
            marker.ns = name;
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.045;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = 0.95F;
            return marker;
        };

        auto before_trajectory = makeTrajectory("trajectory_before", 0, 1.0F, 0.15F, 0.05F);
        auto after_trajectory = makeTrajectory("trajectory_after", 0, 0.05F, 1.0F, 0.15F);

        double max_correction = 0.0;
        for (size_t i = 0; i < optimized.size(); ++i)
            max_correction = std::max(max_correction,
                                      (optimized[i].t - m_initial_poses[i].t).norm());
        max_correction = std::max(max_correction, 1e-9);

        visualization_msgs::msg::Marker magnitude;
        magnitude.header.frame_id = "map";
        magnitude.header.stamp = stamp;
        magnitude.ns = "correction_magnitude";
        magnitude.id = 0;
        magnitude.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        magnitude.action = visualization_msgs::msg::Marker::ADD;
        magnitude.pose.orientation.w = 1.0;
        magnitude.scale.x = m_node_config.correction_marker_size;
        magnitude.scale.y = m_node_config.correction_marker_size;
        magnitude.scale.z = m_node_config.correction_marker_size;

        for (size_t i = 0; i < optimized.size(); ++i)
        {
            geometry_msgs::msg::Point before_point;
            before_point.x = m_initial_poses[i].t.x();
            before_point.y = m_initial_poses[i].t.y();
            before_point.z = m_initial_poses[i].t.z();
            before_trajectory.points.push_back(before_point);

            geometry_msgs::msg::Point after_point;
            after_point.x = optimized[i].t.x();
            after_point.y = optimized[i].t.y();
            after_point.z = optimized[i].t.z();
            after_trajectory.points.push_back(after_point);
            magnitude.points.push_back(after_point);

            const double correction = (optimized[i].t - m_initial_poses[i].t).norm();
            magnitude.colors.push_back(correctionColor(correction / max_correction));
        }
        markers.markers.push_back(before_trajectory);
        markers.markers.push_back(after_trajectory);
        markers.markers.push_back(magnitude);

        const size_t arrow_limit = static_cast<size_t>(std::max(1, m_node_config.max_correction_arrows));
        const size_t stride = std::max<size_t>(1, (optimized.size() + arrow_limit - 1) / arrow_limit);
        int arrow_id = 0;
        for (size_t i = 0; i < optimized.size(); i += stride)
        {
            const Eigen::Vector3d delta = optimized[i].t - m_initial_poses[i].t;
            if (delta.norm() < 1e-5)
                continue;

            visualization_msgs::msg::Marker arrow;
            arrow.header.frame_id = "map";
            arrow.header.stamp = stamp;
            arrow.ns = "correction_arrows_x" +
                       std::to_string(m_node_config.correction_display_scale);
            arrow.id = arrow_id++;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            arrow.pose.orientation.w = 1.0;
            arrow.scale.x = 0.035;
            arrow.scale.y = 0.09;
            arrow.scale.z = 0.12;
            arrow.color = correctionColor(delta.norm() / max_correction);

            geometry_msgs::msg::Point start;
            start.x = m_initial_poses[i].t.x();
            start.y = m_initial_poses[i].t.y();
            start.z = m_initial_poses[i].t.z();
            geometry_msgs::msg::Point end;
            const Eigen::Vector3d display_end =
                m_initial_poses[i].t + m_node_config.correction_display_scale * delta;
            end.x = display_end.x();
            end.y = display_end.y();
            end.z = display_end.z();
            arrow.points = {start, end};
            markers.markers.push_back(arrow);
        }

        m_visualization_pub->publish(markers);
        RCLCPP_INFO(
            this->get_logger(),
            "Published HBA visualization: %lu poses, %d correction arrows (display scale %.1fx, max correction %.4f m)",
            optimized.size(), arrow_id, m_node_config.correction_display_scale, max_correction);
    }

    bool validateOptimization(std::string &message)
    {
        const auto &optimized = m_hba->poses();
        if (optimized.size() != m_initial_poses.size() || optimized.empty())
        {
            message = "pose count mismatch";
            return false;
        }

        double max_translation_correction = 0.0;
        double max_rotation_correction_deg = 0.0;
        double max_step_change = 0.0;
        double max_relative_rotation_change_deg = 0.0;
        size_t max_step_index = 0;
        size_t max_relative_rotation_index = 0;

        for (size_t i = 0; i < optimized.size(); ++i)
        {
            if (!optimized[i].t.allFinite() || !optimized[i].r.allFinite())
            {
                message = "non-finite optimized pose at index " + std::to_string(i);
                return false;
            }

            max_translation_correction = std::max(
                max_translation_correction,
                (optimized[i].t - m_initial_poses[i].t).norm());
            const double rotation_correction_deg =
                std::abs(Eigen::AngleAxisd(
                    m_initial_poses[i].r.transpose() * optimized[i].r).angle()) *
                180.0 / M_PI;
            max_rotation_correction_deg = std::max(
                max_rotation_correction_deg, rotation_correction_deg);

            if (i == 0)
                continue;

            const double initial_step =
                (m_initial_poses[i].t - m_initial_poses[i - 1].t).norm();
            const double optimized_step =
                (optimized[i].t - optimized[i - 1].t).norm();
            const double step_change = std::abs(optimized_step - initial_step);
            if (step_change > max_step_change)
            {
                max_step_change = step_change;
                max_step_index = i - 1;
            }

            const Eigen::Matrix3d initial_relative_rotation =
                m_initial_poses[i - 1].r.transpose() * m_initial_poses[i].r;
            const Eigen::Matrix3d optimized_relative_rotation =
                optimized[i - 1].r.transpose() * optimized[i].r;
            const double relative_rotation_change_deg =
                std::abs(Eigen::AngleAxisd(
                    initial_relative_rotation.transpose() * optimized_relative_rotation).angle()) *
                180.0 / M_PI;
            if (relative_rotation_change_deg > max_relative_rotation_change_deg)
            {
                max_relative_rotation_change_deg = relative_rotation_change_deg;
                max_relative_rotation_index = i - 1;
            }
        }

        // HBA is a refinement stage. Metre-scale discontinuities or large
        // adjacent rotations indicate a corrupted result, not refinement.
        if (max_translation_correction > 5.0 || max_rotation_correction_deg > 30.0 ||
            max_step_change > 0.5 || max_relative_rotation_change_deg > 30.0)
        {
            std::ostringstream stream;
            stream << "max correction=" << max_translation_correction << " m / "
                   << max_rotation_correction_deg << " deg, max adjacent step change="
                   << max_step_change << " m at " << max_step_index << "->"
                   << max_step_index + 1 << ", max adjacent rotation change="
                   << max_relative_rotation_change_deg << " deg at "
                   << max_relative_rotation_index << "->" << max_relative_rotation_index + 1;
            message = stream.str();
            return false;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "HBA validation passed: max adjacent step change=%.4f m, "
            "max adjacent rotation change=%.3f deg",
            max_step_change, max_relative_rotation_change_deg);
        return true;
    }

private:
    NodeConfig m_node_config;
    HBAConfig m_hba_config;
    std::shared_ptr<HBA> m_hba;
    rclcpp::Service<interface::srv::RefineMap>::SharedPtr m_refine_map_srv;
    rclcpp::Service<interface::srv::SavePoses>::SharedPtr m_save_poses_srv;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_before_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_after_cloud_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_visualization_pub;

    pcl::VoxelGrid<pcl::PointXYZI> m_voxel_grid;
    std::vector<Pose> m_initial_poses;
    std::filesystem::path m_maps_path;
    std::mutex m_service_mutex;
    bool m_do_optimize = false;
    rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HBANode>());
    rclcpp::shutdown();
    return 0;
}
