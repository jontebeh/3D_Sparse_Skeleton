#include <libcore/visualizer.hpp>
#include <Eigen/Core>

using namespace open3d;

namespace libcore {

    void Visualizer::InitWindow(const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
        vis_ = std::make_shared<visualization::Visualizer>();
        vis_->CreateVisualizerWindow("Live Skeleton Visualizer", 1600, 900);

        auto o3d_cloud = std::make_shared<geometry::PointCloud>();
        for (const auto& pt : cloud.points) {

            //o3d_cloud->points_.emplace_back(pt.x, pt.y, pt.z);
            // with color
            o3d_cloud->points_.emplace_back(pt.x, pt.y, pt.z);

            // Convert RGB values from 0-255 to 0-1
            Eigen::Vector3d color(
                static_cast<double>(pt.r) / 255.0,
                static_cast<double>(pt.g) / 255.0,
                static_cast<double>(pt.b) / 255.0
            );
            o3d_cloud->colors_.emplace_back(color);
        }
        
        vis_->AddGeometry(o3d_cloud);
    }

    void Visualizer::EnqueueNode(const NodePtr& node) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        node_queue_.push(node);
    }

    void Visualizer::Run() {
        running_ = true;
        while (running_) {
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                while (!node_queue_.empty()) {
                    NodePtr node = node_queue_.front();
                    node_queue_.pop();

                    Eigen::Vector3d new_pos(node->coord(0), node->coord(1), node->coord(2));
                    auto sphere = geometry::TriangleMesh::CreateSphere(0.1);
                    sphere->Translate(new_pos);
                    sphere->PaintUniformColor({1.0, 0.0, 0.0});
                    spheres_.push_back(sphere);
                    node_spheres_[node->id] = sphere;
                    vis_->AddGeometry(sphere, false);

                    // add lines for connections
                    for (const auto &conn: node->connected_nodes) {
                        if (node_spheres_.count(conn->id) == 0) {
                            continue; // Skip invalid connections
                        }
                        if (conn->id == 0) continue; // Skip the root node connection
                        auto line = std::make_shared<geometry::LineSet>();
                        // Add the two endpoints
                        line->points_.push_back(node_spheres_[node->id]->GetCenter());
                        line->points_.push_back(node_spheres_[conn->id]->GetCenter());
                        // Add a line between them
                        line->lines_.push_back(Eigen::Vector2i(0, 1));
                        line->PaintUniformColor({0.0, 1.0, 0.0});
                        vis_->AddGeometry(line, false);
                    }
                }
            }

            vis_->PollEvents();
            vis_->UpdateRender();
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
    }

    void Visualizer::Close() {
        running_ = false;
        vis_->DestroyVisualizerWindow();
    }
}