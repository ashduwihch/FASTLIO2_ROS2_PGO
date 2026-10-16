#include "hba.h"

#include <fstream>
#include <iostream>
#include <stdexcept>

#include <pcl/common/transforms.h>

#include "official/official_hierarchy.hpp"

using OfficialPoint = PointType;

static void official_transform_pointcloud(
    const pcl::PointCloud<OfficialPoint> &input,
    pcl::PointCloud<OfficialPoint> &output,
    const Eigen::Vector3d &translation,
    const Eigen::Quaterniond &rotation)
{
    output.resize(input.size());
    for (size_t i = 0; i < input.size(); ++i)
    {
        const Eigen::Vector3d point(input[i].x, input[i].y, input[i].z);
        const Eigen::Vector3d transformed = rotation * point + translation;
        output[i].x = static_cast<float>(transformed.x());
        output[i].y = static_cast<float>(transformed.y());
        output[i].z = static_cast<float>(transformed.z());
    }
}

static pcl::PointCloud<OfficialPoint>::Ptr official_append_cloud(
    pcl::PointCloud<OfficialPoint>::Ptr destination,
    const pcl::PointCloud<OfficialPoint> &source)
{
    *destination += source;
    return destination;
}

// This is the official hku-mars/HBA computation flow with only ROS1 timing,
// file I/O and point-cloud plumbing adapted for the ROS2 in-memory wrapper.
#include "official/official_core.inc"

struct HBA::Impl
{
};

HBA::HBA(const HBAConfig &config)
    : m_impl(std::make_unique<Impl>()), m_config(config)
{
}

HBA::~HBA() = default;

void HBA::insert(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const Pose &pose)
{
    m_clouds.push_back(cloud);
    m_poses.push_back(pose);
}

void HBA::optimize()
{
    if (m_poses.size() != m_clouds.size())
        throw std::runtime_error("HBA pose/cloud count mismatch");
    if (m_poses.size() < WIN_SIZE)
        throw std::runtime_error("HBA requires at least WIN_SIZE keyframes");

    std::vector<official_hba_types::Pose> official_poses(m_poses.size());
    std::vector<pcl::PointCloud<OfficialPoint>::Ptr> official_clouds(m_clouds.size());

    for (size_t i = 0; i < m_poses.size(); ++i)
    {
        official_poses[i].q = Eigen::Quaterniond(m_poses[i].r);
        official_poses[i].t = m_poses[i].t;

        auto cloud = std::make_shared<pcl::PointCloud<OfficialPoint>>();
        cloud->resize(m_clouds[i]->size());
        for (size_t j = 0; j < m_clouds[i]->size(); ++j)
        {
            (*cloud)[j].x = (*m_clouds[i])[j].x;
            (*cloud)[j].y = (*m_clouds[i])[j].y;
            (*cloud)[j].z = (*m_clouds[i])[j].z;
        }
        official_clouds[i] = cloud;
    }

    OfficialHierarchy hierarchy(
        m_config.total_layer_num,
        m_config.thread_num,
        official_poses,
        official_clouds,
        m_config.down_sample,
        m_config.voxel_size,
        m_config.eigen_ratio,
        m_config.reject_ratio,
        m_config.max_iter);

    for (int level = 0; level < m_config.total_layer_num - 1; ++level)
    {
        std::cout << "---------------------\nOFFICIAL HBA LAYER " << level + 1 << std::endl;
        distribute_thread(hierarchy.layers[level], hierarchy.layers[level + 1]);
        hierarchy.update_next_layer_state(level);
    }

    std::cout << "---------------------\nOFFICIAL HBA GLOBAL BA" << std::endl;
    global_ba(hierarchy.layers[m_config.total_layer_num - 1]);
    hierarchy.pose_graph_optimization();

    const auto &optimized = hierarchy.layers[0].pose_vec;
    if (optimized.size() != m_poses.size())
        throw std::runtime_error("Official HBA returned an unexpected pose count");

    for (size_t i = 0; i < optimized.size(); ++i)
    {
        m_poses[i].r = optimized[i].q.normalized().toRotationMatrix();
        m_poses[i].t = optimized[i].t;
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr HBA::getMapPoints()
{
    auto map = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    for (size_t i = 0; i < m_poses.size(); ++i)
    {
        auto transformed = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::transformPointCloud(
            *m_clouds[i], *transformed, m_poses[i].t, Eigen::Quaterniond(m_poses[i].r));
        *map += *transformed;
    }
    return map;
}

void HBA::writePoses(const std::string &path)
{
    std::ofstream output(path, std::ios::trunc);
    if (!output.is_open())
        throw std::runtime_error("Failed to open HBA pose output: " + path);

    for (const Pose &pose : m_poses)
    {
        const Eigen::Quaterniond q(pose.r);
        output << pose.t.x() << " " << pose.t.y() << " " << pose.t.z() << " "
               << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << "\n";
    }
}
