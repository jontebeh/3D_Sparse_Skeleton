#include <libcore/visualizer.hpp>
#include <Eigen/Core>
#include <libcore/logger.hpp>

using namespace open3d;
namespace gui = open3d::visualization::gui;
namespace rendering = open3d::visualization::rendering;

namespace libcore {

// -------------------------- Helpers --------------------------

void Visualizer::SetupMaterials_() {
    // Unlit points for point cloud
    mat_cloud_ = rendering::MaterialRecord();
    mat_cloud_.shader = "defaultUnlit";
    mat_cloud_.point_size = 2.0f; // adjust to taste

    // Lit mesh for nodes (spheres)
    mat_node_ = rendering::MaterialRecord();
    mat_node_.shader = "defaultLit";

    // Unlit lines for connections
    mat_line_ = rendering::MaterialRecord();
    mat_line_.shader = "unlitLine";
    mat_line_.line_width = 2.0f;

    // Debug spheres
    mat_debug_ = rendering::MaterialRecord();
    mat_debug_.shader = "defaultLit";

    // Facet Meshes
    mat_facet_ = rendering::MaterialRecord();
    mat_facet_.shader = "defaultLitTransparency";
    mat_facet_.base_color = Eigen::Vector4f(0.5f, 0.5f, 0.5f, 0.8f);
}

void Visualizer::AddCloudToScene_() {
    // 0.19: pointer + trailing bool
    scene_->AddGeometry("cloud", cloud_.get(), mat_cloud_, /*add_downsampled*/ false);

    // Fit camera to cloud via SceneWidget (center must be float in 0.19)
    auto bounds = cloud_->GetAxisAlignedBoundingBox();
    scene_widget_->SetupCamera(60.0f, bounds, bounds.GetCenter().cast<float>());
}

// Drain enqueued work on the GUI thread only
void Visualizer::DrainQueuesOnGuiThread_() {
    // Move queued work to locals, then render-add on GUI thread.
    std::queue<NodePtr> nodes_local;
    std::queue<std::pair<Eigen::Vector3d, Eigen::Vector3d>> debug_local;

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        std::swap(nodes_local, node_queue_);
        std::swap(debug_local, debug_node_queue_);
    }

    // Add nodes (spheres + edges)
    while (!nodes_local.empty()) {
        NodePtr node = nodes_local.front();
        nodes_local.pop();

        const Eigen::Vector3d new_pos(node->coord(0), node->coord(1), node->coord(2));

        auto sphere = geometry::TriangleMesh::CreateSphere(0.1);
        sphere->ComputeVertexNormals();
        sphere->Translate(new_pos);

        // Color by state
        if (node->rollbacked) {
            sphere->PaintUniformColor({0.5, 0.5, 0.5}); // gray
        } else {
            sphere->PaintUniformColor({1.0, 0.0, 0.0}); // red
        }

        // Keep alive + map id -> sphere
        node_spheres_[node->debug_id] = sphere;
        scratch_geoms_.push_back(sphere);
        const std::string sphere_name = "node_sphere_" + std::to_string(node->debug_id);
        libcore::debug << "Adding Vis sphere " << sphere_name << std::endl;

        // 0.19: pointer + bool
        scene_->AddGeometry(sphere_name.c_str(), sphere.get(), mat_node_, false);

        libcore::debug << "Adding Vis lines for " << sphere_name << std::endl;
        // Edges to connected nodes (if they already exist)
        for (const auto& conn : node->connected_nodes) {
            if (!conn || conn->debug_id == 0) continue;
            auto it = node_spheres_.find(conn->debug_id);
            if (it == node_spheres_.end()) continue;

            auto line = std::make_shared<geometry::LineSet>();
            line->points_.push_back(node_spheres_[node->debug_id]->GetCenter());
            line->points_.push_back(it->second->GetCenter());
            line->lines_.push_back(Eigen::Vector2i(0, 1));
            line->PaintUniformColor({0.0, 1.0, 0.0}); // green

            scratch_geoms_.push_back(line);
            const std::string edge_name =
                "edge_" + std::to_string(node->debug_id) + "_" + std::to_string(conn->debug_id);
            scene_->AddGeometry(edge_name.c_str(), line.get(), mat_line_, false);
        }

        libcore::debug << "Adding Vis black vertices for " << sphere_name << std::endl;
        // Black vertices (blue)
        for (const auto& v : node->black_vertices) {
            if (!v) continue;
            auto s = geometry::TriangleMesh::CreateSphere(0.02);
            s->ComputeVertexNormals();
            s->Translate(v->coord);
            s->PaintUniformColor({0.0, 0.0, 1.0});
            scratch_geoms_.push_back(s);
            const std::string name = "black_v_" + std::to_string(node->debug_id) + "_" +
                                     std::to_string(reinterpret_cast<uintptr_t>(v.get()));
            libcore::debug << "Adding Vis black vertex at " << v->coord.transpose() << std::endl;
            // check if name already exists
            if (scene_->HasGeometry(name.c_str())) {
                libcore::error << "Geometry with name " << name << " already exists. Skipping." << std::endl;
                continue;
            }
            scene_->AddGeometry(name.c_str(), s.get(), mat_debug_, false);
        }

        libcore::debug << "Adding Vis white vertices for " << sphere_name << std::endl;
        // White vertices (yellow)
        for (const auto& v : node->white_vertices) {
            if (!v) continue;
            auto s = geometry::TriangleMesh::CreateSphere(0.02);
            s->ComputeVertexNormals();
            s->Translate(v->coord);
            s->PaintUniformColor({1.0, 1.0, 0.0});
            scratch_geoms_.push_back(s);
            const std::string name = "white_v_" + std::to_string(node->debug_id) + "_" +
                                     std::to_string(reinterpret_cast<uintptr_t>(v.get()));
            scene_->AddGeometry(name.c_str(), s.get(), mat_debug_, false);
        }

        libcore::debug << "Adding Vis facets for " << sphere_name << std::endl;
        // add facets as mesh
        std::vector<Eigen::Vector3d> vertices;
        std::vector<Eigen::Vector3i> triangles;
        for (const auto& f : node->facets) {
            if (!f) continue;
            // Add vertices
            vertices.push_back(f->vertices[0]->coord);
            vertices.push_back(f->vertices[1]->coord);
            vertices.push_back(f->vertices[2]->coord);
            // Add triangle
            triangles.emplace_back(vertices.size() - 3, vertices.size() - 2, vertices.size() - 1);
        }

        if (!triangles.empty()) {
            auto mesh = std::make_shared<geometry::TriangleMesh>(vertices, triangles);
            mesh->ComputeVertexNormals();
            // set color with alpha
            mesh->PaintUniformColor({0.5, 0.5, 0.5}); // gray w
            scratch_geoms_.push_back(mesh);
            const std::string mesh_name = "facet_mesh_" + std::to_string(node->debug_id);
            //scene_->AddGeometry(mesh_name.c_str(), mesh.get(), mat_facet_, false);
        }

        libcore::debug << "Adding Vis frontiers for " << sphere_name << std::endl;
        for (const auto& f : node->frontiers) {
            std::vector<Eigen::Vector3d> frontier_vertices;
            std::vector<Eigen::Vector3i> frontier_triangles;
            for (const auto& facet : f->facets) {
                if (!facet) continue;
                // Add vertices
                frontier_vertices.push_back(facet->vertices[0]->coord);
                frontier_vertices.push_back(facet->vertices[1]->coord);
                frontier_vertices.push_back(facet->vertices[2]->coord);

                // if 2 points have the same coord, log error
                if (facet->vertices[0]->coord == facet->vertices[1]->coord ||
                    facet->vertices[0]->coord == facet->vertices[2]->coord ||
                    facet->vertices[1]->coord == facet->vertices[2]->coord) {
                    libcore::error << "Found duplicate vertex coordinates in facet for node " << node->debug_id << std::endl;
                    continue;
                }

                // Add triangle
                frontier_triangles.emplace_back(
                    frontier_vertices.size() - 3, 
                    frontier_vertices.size() - 2,
                    frontier_vertices.size() - 1);
            }
            if (!frontier_triangles.empty()) {
                auto frontier_mesh = std::make_shared<geometry::TriangleMesh>(
                    frontier_vertices, frontier_triangles);
                frontier_mesh->ComputeVertexNormals();
                // generate a random color for the frontier mesh
                Eigen::Vector3d frontier_color = Eigen::Vector3d::Random().cwiseAbs();
                frontier_color.normalize();
                frontier_mesh->PaintUniformColor(frontier_color);
                scratch_geoms_.push_back(frontier_mesh);
                const std::string frontier_name = "frontier_mesh_" + std::to_string(node->debug_id) +
                                                  "_" + std::to_string(f->index);
                scene_->AddGeometry(frontier_name.c_str(), frontier_mesh.get(), mat_facet_, false);
            }
        }
    }

    // Debug nodes (colored spheres)
    while (!debug_local.empty()) {
        auto [position, color] = debug_local.front();
        debug_local.pop();

        auto s = geometry::TriangleMesh::CreateSphere(0.04);
        s->ComputeVertexNormals();
        s->Translate(position);
        s->PaintUniformColor(color);
        scratch_geoms_.push_back(s);
        const std::string name = "debug_" + std::to_string(reinterpret_cast<uintptr_t>(s.get()));
        libcore::debug << "Adding Debug sphere " << name << std::endl;
        scene_->AddGeometry(name.c_str(), s.get(), mat_debug_, false);
    }
}

// -------------------------- Public API --------------------------

void Visualizer::InitWindow(const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
    // 0.19: Initialize takes (int, const char**), and there is no IsRunning()
    int argc = 0;
    const char** argv = nullptr;
    gui::Application::GetInstance().Initialize(argc, argv);

    // Build point cloud
    cloud_ = std::make_shared<geometry::PointCloud>();
    cloud_->points_.reserve(cloud.size());
    cloud_->colors_.reserve(cloud.size());
    for (const auto& pt : cloud.points) {
        cloud_->points_.emplace_back(pt.x, pt.y, pt.z);
        cloud_->colors_.emplace_back(double(pt.r) / 255.0,
                                     double(pt.g) / 255.0,
                                     double(pt.b) / 255.0);
    }

    SetupMaterials_();

    // 0.19: Create a window as shared_ptr and register it
    window_ = std::make_shared<gui::Window>("Live Skeleton Visualizer", 1600, 900);
    gui::Application::GetInstance().AddWindow(window_);

    // SceneWidget + Open3DScene (shared_ptr)
    scene_widget_ = std::make_shared<gui::SceneWidget>();
    scene_ = std::make_shared<rendering::Open3DScene>(window_->GetRenderer());
    scene_widget_->SetScene(scene_);

    window_->AddChild(scene_widget_);

    // Fill the window initially. (No resize callback needed to compile.)
    scene_widget_->SetFrame(window_->GetContentRect());

    // Close handling
    window_->SetOnClose([this]() {
        running_ = false;
        return true; // allow close
    });

    // Add base cloud and set camera
    AddCloudToScene_();
}

void Visualizer::EnqueueNode(const NodePtr& node) {
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        node_queue_.push(node);
    }
    if (window_) {
        // PostToMainThread takes a raw Window*
        gui::Application::GetInstance().PostToMainThread(window_.get(),
            [this]() { this->DrainQueuesOnGuiThread_(); });
    }
}

void Visualizer::EnqueueDebugNode(const Eigen::Vector3d& position,
                                  const Eigen::Vector3d& color) {
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        debug_node_queue_.emplace(position, color);
    }
    if (window_) {
        gui::Application::GetInstance().PostToMainThread(window_.get(),
            [this]() { this->DrainQueuesOnGuiThread_(); });
    }
}

void Visualizer::Run() {
    running_ = true;
    gui::Application::GetInstance().Run();
}

void Visualizer::Close() {
    running_ = false;
    if (window_) {
        gui::Application::GetInstance().PostToMainThread(window_.get(),
            [w = window_]() { w->Close(); });
    }
}

} // namespace libcore
