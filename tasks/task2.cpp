#include <vector>
#include <string>
#include <algorithm>
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


void task2() {

    polyscope::init();
    polyscope::view::setUpDir(polyscope::UpDir::ZUp);

    std::string robot_dir = "robot/franka_description";
    std::string urdf_path = robot_dir + "/robot.urdf";
    pinocchio::Model model;
    pinocchio::GeometryModel visual_model;
    pinocchio::urdf::buildModel(urdf_path, model);
    pinocchio::urdf::buildGeom(model, urdf_path, pinocchio::VISUAL, visual_model, robot_dir);
    pinocchio::Data data(model);
    pinocchio::GeometryData visual_data(visual_model);

    // TODO 1: Define the geometry and collision pair of the robot.
    pinocchio::GeometryModel collision_model;
    pinocchio::urdf::buildGeom(model, urdf_path, pinocchio::COLLISION, collision_model, robot_dir);
    collision_model.addAllCollisionPairs();
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
        std::cout << obj.name << std::endl;
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

    auto is_near_neighbor_pair = [&](int pair_idx) {
        const auto& collision_pair = collision_model.collisionPairs[pair_idx];
        const auto& goA = collision_model.geometryObjects[collision_pair.first];
        const auto& goB = collision_model.geometryObjects[collision_pair.second];
        pinocchio::JointIndex jA = goA.parentJoint;
        pinocchio::JointIndex jB = goB.parentJoint;

        // TODO 2: Remove the collision detection between self nodes, parent-child nodes, and brother nodes.
        if (jA == jB) return true;
        if (jA > 0 && jB > 0) {
            if (model.parents[jA] == jB || model.parents[jB] == jA) return true;
            if (model.parents[jA] == model.parents[jB]) return true;
        }
        return false;
    };

    bool in_collision = false;
    std::vector<std::pair<std::string, std::string>> colliding_pairs;
    polyscope::state::userCallback = [&]() {
        ImGui::Text("Task2: Self-collision detection");
        ImGui::Separator();

        if (ImGui::Button("Reset to neutral")) {
            q = pinocchio::neutral(model);
            for (int i = 0; i < 7; ++i) q_ui[i] = static_cast<float>(q[i]);
            gripper_width = 0.04f;
        }

        ImGui::Separator();
        ImGui::Text("Arm joints");
        for (int j = 0; j < 7; ++j) {
            std::string label = "q" + std::to_string(j);
            ImGui::SliderFloat(label.c_str(), &q_ui[j], q_min[j], q_max[j]);
        }

        ImGui::Separator();
        ImGui::Text("Gripper");
        ImGui::SliderFloat("gripper_width", &gripper_width, 0.0f, 0.04f);
        for (int j = 0; j < 7; ++j) q[j] = q_ui[j];
        if (finger1_qidx >= 0) q[finger1_qidx] = gripper_width;
        if (finger2_qidx >= 0) q[finger2_qidx] = gripper_width;
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

        // TODO 3: Compute collision and obtain the result of collision.
        //  Remember to ignore the collision of the near neighbor nodes.
        pinocchio::computeCollisions(model, data, collision_model, collision_data, q, false);
        colliding_pairs.clear();
        for (int k = 0; k < (int)collision_model.collisionPairs.size(); ++k) {
            if (!collision_data.collisionResults[k].isCollision()) continue;
            if (is_near_neighbor_pair(k)) continue;
            const auto& collision_pair = collision_model.collisionPairs[k];
            const std::string& nameA = collision_model.geometryObjects[collision_pair.first].name;
            const std::string& nameB = collision_model.geometryObjects[collision_pair.second].name;
            colliding_pairs.push_back({nameA, nameB});
        }

        in_collision = !colliding_pairs.empty();

        ImGui::Separator();
        if (in_collision) {
            ImGui::TextColored(ImVec4(1.f, 0.f, 0.f, 1.f), "In collision");
        } else {
            ImGui::TextColored(ImVec4(0.f, 1.f, 0.f, 1.f), "Collision-free");
        }

        if (!colliding_pairs.empty()) {
            ImGui::Text("Colliding pairs:");
            int max_show = std::min((int)colliding_pairs.size(), 10);
            for (int i = 0; i < max_show; ++i) {
                ImGui::BulletText("%s <-> %s",
                                  colliding_pairs[i].first.c_str(),
                                  colliding_pairs[i].second.c_str());
            }
            if ((int)colliding_pairs.size() > max_show) {
                ImGui::Text("... and %d more", (int)colliding_pairs.size() - max_show);
            }
        }
    };

    polyscope::show();
}