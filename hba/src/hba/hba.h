#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct Pose
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix3d r{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d t{Eigen::Vector3d::Zero()};
};

struct HBAConfig
{
    int total_layer_num = 3;
    int thread_num = 16;
    int max_iter = 10;
    double down_sample = 0.1;
    double voxel_size = 4.0;
    double eigen_ratio = 0.1;
    double reject_ratio = 0.05;
};

// ROS2-facing wrapper around the official hku-mars/HBA optimization core.
// The node/service API and the PGO map format remain unchanged.
class HBA
{
public:
    explicit HBA(const HBAConfig &config);
    ~HBA();

    void insert(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const Pose &pose);
    void optimize();

    std::vector<Pose> &poses() { return m_poses; }
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clouds() { return m_clouds; }

    pcl::PointCloud<pcl::PointXYZI>::Ptr getMapPoints();
    void writePoses(const std::string &path);

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
    HBAConfig m_config;
    std::vector<Pose> m_poses;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> m_clouds;
};
