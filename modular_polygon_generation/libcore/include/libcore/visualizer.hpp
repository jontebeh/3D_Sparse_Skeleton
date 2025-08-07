#pragma once

#include <memory>
#include <vector>
#include <mutex>
#include <queue>
#include <open3d/Open3D.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <libcore/data_types/node.hpp>

namespace libcore {
    class Visualizer {
    public:
        bool isInitialized() const {
            return vis_ != nullptr;
        }

        // Runtime visualization (optional later)
        void InitWindow(const pcl::PointCloud<pcl::PointXYZRGB>& cloud);
        void EnqueueNode(const NodePtr& node);
        void EnqueueDebugNode(const Eigen::Vector3d& position, const Eigen::Vector3d& color);
        void Run(); // starts the main loop (blocking!)
        void Close();   // optional
    private:
        std::shared_ptr<open3d::visualization::Visualizer> vis_;
        std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> spheres_;
        std::unordered_map<int, std::shared_ptr<open3d::geometry::TriangleMesh>> node_spheres_;
        std::queue<NodePtr> node_queue_;
        std::queue<std::pair<Eigen::Vector3d, Eigen::Vector3d>> debug_node_queue_;
        std::mutex queue_mutex_;
        bool running_ = false;
    };
}
