#pragma once
#include <memory>
#include <vector>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <thread>
#include <atomic>

#include <open3d/Open3D.h>
#include <open3d/visualization/gui/Application.h>
#include <open3d/visualization/gui/SceneWidget.h>
#include <open3d/visualization/rendering/MaterialRecord.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <libcore/data_types/node.hpp>

namespace libcore {

class Visualizer {
public:
    bool isInitialized() const { return window_ != nullptr; }

    // Runtime visualization (optional later)
    void InitWindow(const pcl::PointCloud<pcl::PointXYZRGB>& cloud);
    void EnqueueNode(const NodePtr& node);
    void EnqueueDebugNode(const Eigen::Vector3d& position, const Eigen::Vector3d& color);
    void Run();  // blocking: starts GUI loop
    void Close();

private:
    // --- GUI / rendering ---
    std::shared_ptr<open3d::visualization::gui::SceneWidget> scene_widget_;
    std::shared_ptr<open3d::visualization::gui::Window> window_;
    std::shared_ptr<open3d::visualization::rendering::Open3DScene> scene_;

    // --- Base content ---
    std::shared_ptr<open3d::geometry::PointCloud> cloud_;

    // --- Materials ---
    open3d::visualization::rendering::MaterialRecord mat_cloud_;
    open3d::visualization::rendering::MaterialRecord mat_node_;
    open3d::visualization::rendering::MaterialRecord mat_line_;
    open3d::visualization::rendering::MaterialRecord mat_debug_;
    open3d::visualization::rendering::MaterialRecord mat_facet_;

    // --- Book-keeping for nodes/lines ---
    std::unordered_map<int, std::shared_ptr<open3d::geometry::TriangleMesh>> node_spheres_;
    std::vector<std::shared_ptr<open3d::geometry::Geometry3D>> scratch_geoms_; // keep alive

    // --- Queues (producer threads) ---
    std::queue<NodePtr> node_queue_;
    std::queue<std::pair<Eigen::Vector3d, Eigen::Vector3d>> debug_node_queue_;
    std::mutex queue_mutex_;

    // --- Lifecycle ---
    std::atomic<bool> running_{false};

    // --- Helpers (GUI thread only) ---
    void DrainQueuesOnGuiThread_();
    void SetupMaterials_();
    void AddCloudToScene_();
};

} // namespace libcore
