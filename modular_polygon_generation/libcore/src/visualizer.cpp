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

    void Visualizer::EnqueueDebugNode(const Eigen::Vector3d& position, const Eigen::Vector3d& color) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        debug_node_queue_.emplace(std::make_pair(position, color));
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
                    sphere->PaintUniformColor({1.0, 0.0, 0.0}); // Red for nodes
                    if (node->rollbacked) {
                        sphere->PaintUniformColor({0.5, 0.5, 0.5}); // Gray for rollbacked nodes
                    }
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

                    // display white and black vertices
                    for (const auto &v : node->black_vertices) {
                        auto sphere = geometry::TriangleMesh::CreateSphere(0.05);
                        sphere->Translate(v->coord);
                        sphere->PaintUniformColor({0.0, 0.0, 1.0}); // Blue for black vertices
                        if (node->debug_id == 317) vis_->AddGeometry(sphere, false);
                        if (node->debug_id == 349) vis_->AddGeometry(sphere, false);
                        if (node->debug_id == 348) vis_->AddGeometry(sphere, false);
                        //vis_->AddGeometry(sphere, false);
                    }

                    for (const auto &v : node->white_vertices) {
                        auto sphere = geometry::TriangleMesh::CreateSphere(0.05);
                        sphere->Translate(v->coord);
                        sphere->PaintUniformColor({1.0, 1.0, 0.0}); // Yellow for white vertices
                        if (node->debug_id == 317) vis_->AddGeometry(sphere, false);
                        if (node->debug_id == 349) vis_->AddGeometry(sphere, false);
                        if (node->debug_id == 348) vis_->AddGeometry(sphere, false);
                        //vis_->AddGeometry(sphere, false);
                    }

                    std::vector<Eigen::Vector3d> facet_points;
                    std::vector<Eigen::Vector3i> facet_triangles;

                    for (const auto &f : node->facets) {
                        facet_points.push_back(f->vertices[0]->coord);
                        facet_points.push_back(f->vertices[1]->coord);
                        facet_points.push_back(f->vertices[2]->coord);
                    }

                    for (size_t i = 0; i < facet_points.size(); i += 3) {
                        facet_triangles.push_back({static_cast<int>(i), static_cast<int>(i + 1), static_cast<int>(i + 2)});
                    }

                    auto mesh = std::make_shared<geometry::TriangleMesh>();
                    mesh->vertices_ = facet_points;
                    mesh->triangles_ = facet_triangles;
                    mesh->PaintUniformColor({0.0, 1.0, 1.0}); // Cyan for facets
                    vis_->AddGeometry(mesh, false);

                }

                while (!debug_node_queue_.empty()) {
                    Eigen::Vector3d position = debug_node_queue_.front().first;
                    debug_node_queue_.pop();
                    auto sphere = geometry::TriangleMesh::CreateSphere(0.06);
                    sphere->Translate(position);
                    sphere->PaintUniformColor(debug_node_queue_.front().second); // Use the color from the queue
                    spheres_.push_back(sphere);
                    vis_->AddGeometry(sphere, false);   
                }
            }

            vis_->PollEvents();
            vis_->UpdateRender();
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }

        vis_->DestroyVisualizerWindow();
    }

    void Visualizer::Close() {
        running_ = false;
    }
}