#include <vector>
#include <string>
#include <random>
#include <limits>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <unordered_set>
#include <Eigen/Geometry>
#include <igl/readOBJ.h>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include "imgui.h"

struct RRTNode {
    Eigen::VectorXd q;
    int parent;
};

static std::vector<Eigen::VectorXd> build_path(const std::vector<RRTNode>& tree, int node_idx, const Eigen::VectorXd& q_goal)
{
    std::vector<Eigen::VectorXd> rev_path;
    int cur = node_idx;
    while (cur >= 0) {
        rev_path.push_back(tree[cur].q);
        cur = tree[cur].parent;
    }
    std::reverse(rev_path.begin(), rev_path.end());
    rev_path.push_back(q_goal);
    return rev_path;
}

static double compute_path_length(const std::vector<Eigen::VectorXd>& path) {
    if (path.size() < 2) return 0.0;
    double L = 0.0;
    for (int i = 0; i + 1 < (int)path.size(); ++i) {
        L += (path[i + 1] - path[i]).norm();
    }
    return L;
}

static int nearest_node(const std::vector<RRTNode>& tree, const Eigen::VectorXd& q_rand)
{
    int near_id = -1;
    double best_dist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < (int)tree.size(); ++i) {
        double d = (tree[i].q - q_rand).squaredNorm();
        if (d < best_dist) {
            best_dist = d;
            near_id = i;
        }
    }
    return near_id;
}

static Eigen::VectorXd steer(const Eigen::VectorXd& q_near, const Eigen::VectorXd& q_rand, const float q_min[7], const float q_max[7], double step_size)
{
    Eigen::VectorXd dir = q_rand - q_near;
    double dir_norm = dir.norm();
    if (dir_norm < 1e-9) return q_near;
    dir /= dir_norm;
    Eigen::VectorXd q_new = q_near + step_size * dir;
    for (int j = 0; j < 7; ++j) {
        q_new[j] = std::max((double)q_min[j], std::min((double)q_max[j], q_new[j]));
    }
    return q_new;
}

static std::vector<Eigen::VectorXd> rrt_single(
    const Eigen::VectorXd& q_start,
    const Eigen::VectorXd& q_goal,
    const float q_min[7],
    const float q_max[7],
    pinocchio::Model& model,
    pinocchio::Data& data,
    pinocchio::GeometryModel& collision_model,
    pinocchio::GeometryData& collision_data,
    int finger1_qidx,
    int finger2_qidx,
    float gripper_width,
    int rng_seed)
{
    const int max_iters = 2500;
    const double step_size = 0.20;
    const double goal_bias = 0.10;
    const double goal_thresh = 0.20;
    const int edge_checks = 4;

    auto ignore_pair = [&](int pair_idx) {
        const auto& cp = collision_model.collisionPairs[pair_idx];
        const auto& goA = collision_model.geometryObjects[cp.first];
        const auto& goB = collision_model.geometryObjects[cp.second];
        pinocchio::JointIndex jA = goA.parentJoint;
        pinocchio::JointIndex jB = goB.parentJoint;
        if (jA == jB) return true;
        if (jA > 0 && jB > 0) {
            if (model.parents[jA] == jB || model.parents[jB] == jA) return true;
            if (model.parents[jA] == model.parents[jB]) return true;
        }
        return false;
    };

    std::vector<bool> ignore_pairs(collision_model.collisionPairs.size(), false);
    for (int k = 0; k < (int)collision_model.collisionPairs.size(); ++k) {
        ignore_pairs[k] = ignore_pair(k);
    }

    auto valid_node = [&](const Eigen::VectorXd& q7) {
        Eigen::VectorXd q_full = pinocchio::neutral(model);
        for (int j = 0; j < 7; ++j) q_full[j] = q7[j];

        if (finger1_qidx >= 0) q_full[finger1_qidx] = gripper_width;
        if (finger2_qidx >= 0) q_full[finger2_qidx] = gripper_width;

        pinocchio::computeCollisions(model, data, collision_model, collision_data, q_full, false);

        for (int k = 0; k < (int)collision_model.collisionPairs.size(); ++k) {
            if (!collision_data.collisionResults[k].isCollision()) continue;
            if (ignore_pairs[k]) continue;
            return false;
        }
        return true;
    };

    auto valid_edge = [&](const Eigen::VectorXd& qa, const Eigen::VectorXd& qb) {
        for (int i = 0; i <= edge_checks; ++i) {
            double t = double(i) / double(edge_checks);
            Eigen::VectorXd qm = (1.0 - t) * qa + t * qb;
            if (!valid_node(qm)) return false;
        }
        return true;
    };

    std::mt19937 rng(rng_seed);
    std::uniform_real_distribution<double> uni01(0.0, 1.0);
    std::uniform_real_distribution<double> joint_dists[7];
    for (int j = 0; j < 7; ++j) {
        joint_dists[j] = std::uniform_real_distribution<double>(q_min[j], q_max[j]);
    }

    auto sample_q = [&]() {
        if (uni01(rng) < goal_bias) return q_goal;

        Eigen::VectorXd q_rand(7);
        for (int j = 0; j < 7; ++j) {
            q_rand[j] = joint_dists[j](rng);
        }
        return q_rand;
    };

    std::vector<Eigen::VectorXd> empty_path;
    if (!valid_node(q_start)) return empty_path;
    if (!valid_node(q_goal)) return empty_path;

    std::vector<RRTNode> tree;
    tree.push_back({q_start, -1});

    for (int it = 0; it < max_iters; ++it) {
        Eigen::VectorXd q_rand = sample_q();

        // TODO 1: Please complete the RRT main loop, using the functions:
        //  - nearest_node()
        //  - steer()
        //  - valid_edge()
        //  - build_path()




    }

    return empty_path;
}

static std::vector<std::vector<Eigen::VectorXd>> rrt_multi(
    const Eigen::VectorXd& q_start,
    const Eigen::VectorXd& q_goal,
    const float q_min[7],
    const float q_max[7],
    pinocchio::Model& model,
    pinocchio::Data& data,
    pinocchio::GeometryModel& collision_model,
    pinocchio::GeometryData& collision_data,
    int finger1_qidx,
    int finger2_qidx,
    float gripper_width)
{
    const int desired_num_paths = 6;
    const int max_restarts = 15;
    std::vector<std::vector<Eigen::VectorXd>> paths;
    std::random_device rd;
    int base_seed = (int)rd();
    for (int r = 0; r < max_restarts; ++r) {
        int seed = base_seed + 97 * r;
        auto path = rrt_single(
            q_start, q_goal,
            q_min, q_max,
            model, data,
            collision_model, collision_data,
            finger1_qidx, finger2_qidx,
            gripper_width,
            seed
        );
        if (path.empty()) continue;
        paths.push_back(path);
        if ((int)paths.size() >= desired_num_paths) break;
    }
    return paths;
}

void task3() {

    polyscope::init();
    polyscope::view::setUpDir(polyscope::UpDir::ZUp);

    std::string robot_dir = "robot/franka_description";
    std::string urdf_path = robot_dir + "/robot.urdf";
    pinocchio::Model model;
    pinocchio::GeometryModel visual_model;
    pinocchio::GeometryModel collision_model;
    pinocchio::urdf::buildModel(urdf_path, model);
    pinocchio::urdf::buildGeom(model, urdf_path, pinocchio::VISUAL, visual_model, robot_dir);
    pinocchio::urdf::buildGeom(model, urdf_path, pinocchio::COLLISION, collision_model, robot_dir);
    collision_model.addAllCollisionPairs();
    int robot_geom_count = (int)collision_model.geometryObjects.size();
    Eigen::Vector3d wall_size(0.02, 0.35, 0.22);
    Eigen::Vector3d wall_t(0.42, 0.25, 0.65);
    pinocchio::SE3 wall_pose(Eigen::Matrix3d::Identity(), wall_t);
    std::shared_ptr<coal::CollisionGeometry> wall_geom = std::make_shared<coal::Box>(wall_size.x(), wall_size.y(), wall_size.z());
    pinocchio::GeometryObject wall_obj("wall_collision", 0, wall_pose, wall_geom);
    pinocchio::GeomIndex wall_id = collision_model.addGeometryObject(wall_obj);

    for (int i = 0; i < robot_geom_count; ++i) {
        // TODO 2: Add the wall object to the collision model for collision detection.



    }
    pinocchio::Data data(model);
    pinocchio::GeometryData visual_data(visual_model);
    pinocchio::GeometryData collision_data(collision_model);
    Eigen::VectorXd q = pinocchio::neutral(model);
    float q_ui[7];
    for (int i = 0; i < 7; ++i) q_ui[i] = static_cast<float>(q[i]);
    const float q_min[7] = {-2.9f, -1.8f, -2.9f, -3.1f, -2.9f, -0.1f, -2.9f};
    const float q_max[7] = { 2.9f,  1.8f,  2.9f,  0.1f,  2.9f,  3.7f,  2.9f};

    pinocchio::JointIndex finger1_id = model.getJointId("fr3_finger_joint1");
    pinocchio::JointIndex finger2_id = model.getJointId("fr3_finger_joint2");
    int finger1_qidx = (finger1_id > 0) ? model.joints[finger1_id].idx_q() : -1;
    int finger2_qidx = (finger2_id > 0) ? model.joints[finger2_id].idx_q() : -1;
    float gripper_width = 0.04f;

    std::vector<Eigen::MatrixXd> V_locals;
    std::vector<polyscope::SurfaceMesh*> meshes;
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::updateGeometryPlacements(model, data, visual_model, visual_data);
    for (int i = 0; i < (int)visual_model.geometryObjects.size(); ++i) {
        const auto& obj = visual_model.geometryObjects[i];
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        igl::readOBJ(obj.meshPath, V, F);
        pinocchio::SE3 M = visual_data.oMg[i];
        Eigen::Matrix3d R = M.rotation();
        Eigen::Vector3d t = M.translation();
        Eigen::MatrixXd Vw = (V * R.transpose()).rowwise() + t.transpose();
        auto* ms = polyscope::registerSurfaceMesh(obj.name + "_" + std::to_string(i), Vw, F);
        V_locals.push_back(V);
        meshes.push_back(ms);
    }

    std::vector<std::array<double,3>> cubeV = {
        {-0.5,-0.5,-0.5},{ 0.5,-0.5,-0.5},{ 0.5, 0.5,-0.5},{-0.5, 0.5,-0.5},
        {-0.5,-0.5, 0.5},{ 0.5,-0.5, 0.5},{ 0.5, 0.5, 0.5},{-0.5, 0.5, 0.5}
    };
    std::vector<std::array<int,3>> cubeF = {
        {0,1,2},{0,2,3},
        {4,5,6},{4,6,7},
        {0,1,5},{0,5,4},
        {2,3,7},{2,7,6},
        {1,2,6},{1,6,5},
        {0,3,7},{0,7,4}
    };
    Eigen::Vector3d cube_t(0.55, 0.20, 0.60);
    double cube_size = 0.04;
    Eigen::Matrix3d cube_R =
        (Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(M_PI / 6.0, Eigen::Vector3d::UnitY())).toRotationMatrix();
    for (auto& v : cubeV) {
        Eigen::Vector3d p(v[0], v[1], v[2]);
        p = cube_size * (cube_R * p) + cube_t;
        v = {p.x(), p.y(), p.z()};
    }
    auto* cube = polyscope::registerSurfaceMesh("target_cube", cubeV, cubeF);
    std::vector<std::array<double,3>> wallV = {
        {-0.5,-0.5,-0.5},{ 0.5,-0.5,-0.5},{ 0.5, 0.5,-0.5},{-0.5, 0.5,-0.5},
        {-0.5,-0.5, 0.5},{ 0.5,-0.5, 0.5},{ 0.5, 0.5, 0.5},{-0.5, 0.5, 0.5}
    };
    std::vector<std::array<int,3>> wallF = {
        {0,1,2},{0,2,3},
        {4,5,6},{4,6,7},
        {0,1,5},{0,5,4},
        {2,3,7},{2,7,6},
        {1,2,6},{1,6,5},
        {0,3,7},{0,7,4}
    };
    Eigen::Matrix3d wall_R = Eigen::Matrix3d::Identity();
    for (auto& v : wallV) {
        Eigen::Vector3d p(v[0], v[1], v[2]);
        p = wall_R * wall_size.asDiagonal() * p + wall_t;
        v = {p.x(), p.y(), p.z()};
    }
    auto* wall = polyscope::registerSurfaceMesh("obstacle_wall", wallV, wallF);

    Eigen::VectorXd q_start(7);
    Eigen::VectorXd q_goal(7);
    for (int j = 0; j < 7; ++j) {
        q_start[j] = q[j];
        q_goal[j] = q[j];
    }
    bool have_start = false;
    bool have_goal = false;
    std::vector<std::vector<Eigen::VectorXd>> planned_paths;
    int selected_path_idx = 0;
    int selected_frame_idx = 0;
    std::string status = "Set start and goal, then click Plan.";

    auto set_robot_to_q7 = [&](const Eigen::VectorXd& q7) {
        for (int j = 0; j < 7; ++j) {
            q_ui[j] = (float)q7[j];
            q[j] = q7[j];
        }
        if (finger1_qidx >= 0) q[finger1_qidx] = gripper_width;
        if (finger2_qidx >= 0) q[finger2_qidx] = gripper_width;
    };

    polyscope::state::userCallback = [&]() {
        ImGui::Text("Task3: Motion Planning with RRT");
        ImGui::Separator();
        if (ImGui::Button("Reset to neutral")) {
            q = pinocchio::neutral(model);
            for (int i = 0; i < 7; ++i) q_ui[i] = static_cast<float>(q[i]);
            gripper_width = 0.04f;
            planned_paths.clear();
            selected_path_idx = 0;
            selected_frame_idx = 0;
            have_start = false;
            have_goal = false;
            status = "Reset.";
        }

        ImGui::Separator();
        ImGui::Text("Current configuration");
        for (int j = 0; j < 7; ++j) {
            std::string label = "q" + std::to_string(j);
            ImGui::SliderFloat(label.c_str(), &q_ui[j], q_min[j], q_max[j]);
        }
        ImGui::SliderFloat("gripper_width", &gripper_width, 0.0f, 0.04f);
        for (int j = 0; j < 7; ++j) q[j] = q_ui[j];
        if (finger1_qidx >= 0) q[finger1_qidx] = gripper_width;
        if (finger2_qidx >= 0) q[finger2_qidx] = gripper_width;

        ImGui::Separator();
        if (ImGui::Button("Set Start")) {
            for (int j = 0; j < 7; ++j) q_start[j] = q_ui[j];
            have_start = true;
            status = "Start set.";
        }

        ImGui::SameLine();
        if (ImGui::Button("Set Goal")) {
            for (int j = 0; j < 7; ++j) q_goal[j] = q_ui[j];
            have_goal = true;
            status = "Goal set.";
        }

        ImGui::SameLine();
        if (ImGui::Button("Plan")) {
            planned_paths.clear();
            selected_path_idx = 0;
            selected_frame_idx = 0;
            if (!have_start || !have_goal) {
                status = "Please set both start and goal first.";
            } else {
                // TODO 3: Call the rrt_multi() function to run motion planning.



                if (planned_paths.empty()) {
                    status = "No valid path found.";
                } else {
                    set_robot_to_q7(planned_paths[0][0]);
                    status = "Found " + std::to_string((int)planned_paths.size()) + " candidate path(s).";
                }
            }
        }

        ImGui::Separator();
        ImGui::Text("Start set: %s", have_start ? "yes" : "no");
        ImGui::Text("Goal set: %s", have_goal ? "yes" : "no");
        ImGui::Text("Candidate paths: %d", (int)planned_paths.size());
        ImGui::TextWrapped("Status: %s", status.c_str());

        if (!planned_paths.empty()) {
            int num_paths = (int)planned_paths.size();
            if (selected_path_idx < 0) selected_path_idx = 0;
            if (selected_path_idx >= num_paths) selected_path_idx = num_paths - 1;
            ImGui::Separator();
            ImGui::Text("Trajectory");
            if (ImGui::Button("-")) {
                if (selected_path_idx > 0) {
                    selected_path_idx--;
                    selected_frame_idx = 0;
                    set_robot_to_q7(planned_paths[selected_path_idx][0]);
                }
            }
            ImGui::SameLine();
            ImGui::Text("%d / %d", selected_path_idx, num_paths - 1);
            ImGui::SameLine();
            if (ImGui::Button("+")) {
                if (selected_path_idx < num_paths - 1) {
                    selected_path_idx++;
                    selected_frame_idx = 0;
                    set_robot_to_q7(planned_paths[selected_path_idx][0]);
                }
            }
            const auto& path = planned_paths[selected_path_idx];
            int max_frame = std::max(0, (int)path.size() - 1);
            if (selected_frame_idx < 0) selected_frame_idx = 0;
            if (selected_frame_idx > max_frame) selected_frame_idx = max_frame;
            ImGui::Text("Nodes: %d", (int)path.size());
            ImGui::Text("Length: %.3f", compute_path_length(path));
            bool frame_changed = ImGui::SliderInt("Frame", &selected_frame_idx, 0, max_frame);
            if (frame_changed) {
                set_robot_to_q7(path[selected_frame_idx]);
            }
            if (ImGui::Button("Show start")) {
                selected_frame_idx = 0;
                set_robot_to_q7(path[0]);
            }
            ImGui::SameLine();
            if (ImGui::Button("Show goal")) {
                selected_frame_idx = max_frame;
                set_robot_to_q7(path[max_frame]);
            }
        }

        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        pinocchio::updateGeometryPlacements(model, data, visual_model, visual_data);

        for (int i = 0; i < (int)meshes.size(); ++i) {
            pinocchio::SE3 M = visual_data.oMg[i];
            Eigen::Matrix3d R = M.rotation();
            Eigen::Vector3d t = M.translation();
            Eigen::MatrixXd Vw = (V_locals[i] * R.transpose()).rowwise() + t.transpose();
            meshes[i]->updateVertexPositions(Vw);
        }
    };

    polyscope::show();
}