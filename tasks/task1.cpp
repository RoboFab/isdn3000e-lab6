#include <string>
#include <memory>
#include <vector>
#include <array>
#include <Eigen/Core>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include "imgui.h"


void task1() {

    polyscope::init();
    polyscope::view::setUpDir(polyscope::UpDir::ZUp);

    double sx = 0.4, sy = 0.4, sz = 0.4;
    std::vector<std::array<double, 3>> V = {
        {-sx/2, -sy/2, -sz/2},
        { sx/2, -sy/2, -sz/2},
        { sx/2,  sy/2, -sz/2},
        {-sx/2,  sy/2, -sz/2},
        {-sx/2, -sy/2,  sz/2},
        { sx/2, -sy/2,  sz/2},
        { sx/2,  sy/2,  sz/2},
        {-sx/2,  sy/2,  sz/2}
    };
    std::vector<std::array<int, 3>> F = {
        {0, 1, 2}, {0, 2, 3},
        {4, 5, 6}, {4, 6, 7},
        {0, 1, 5}, {0, 5, 4},
        {1, 2, 6}, {1, 6, 5},
        {2, 3, 7}, {2, 7, 6},
        {3, 0, 4}, {3, 4, 7}
    };
    Eigen::Vector3d t1(0.0, 0.0, 0.0);
    Eigen::Vector3d t2(0.6, 0.0, 0.0);
    std::vector<std::array<double, 3>> V1w = V;
    std::vector<std::array<double, 3>> V2w = V;
    for (auto& p : V1w) {
        p[0] += t1[0];
        p[1] += t1[1];
        p[2] += t1[2];
    }
    for (auto& p : V2w) {
        p[0] += t2[0];
        p[1] += t2[1];
        p[2] += t2[2];
    }
    auto* box1 = polyscope::registerSurfaceMesh("box 1", V1w, F);
    auto* box2 = polyscope::registerSurfaceMesh("box 2", V2w, F);

    pinocchio::Model model;
    pinocchio::Data data(model);
    pinocchio::GeometryModel geom_model;
    auto geom1 = std::make_shared<coal::Box>(sx, sy, sz);
    auto geom2 = std::make_shared<coal::Box>(sx, sy, sz);
    pinocchio::SE3 pose1 = pinocchio::SE3::Identity();
    pose1.translation() = t1;
    pinocchio::SE3 pose2 = pinocchio::SE3::Identity();
    pose2.translation() = t2;

    // TODO 1: Define geometry and collision pair of the two boxes.
    pinocchio::GeometryObject obj1("box1", 0, pose1, geom1);
    pinocchio::GeometryObject obj2("box2", 0, pose2, geom2);
    pinocchio::GeomIndex id1 = geom_model.addGeometryObject(obj1);
    pinocchio::GeomIndex id2 = geom_model.addGeometryObject(obj2);
    geom_model.addCollisionPair(pinocchio::CollisionPair(id1, id2));
    pinocchio::GeometryData geom_data(geom_model);


    Eigen::VectorXd q(0);
    float tx = 0.6f;
    float ty = 0.0f;
    float tz = 0.0f;
    bool in_collision = false;
    polyscope::state::userCallback = [&]() {
        ImGui::Text("Task 1: Box collision detection");
        ImGui::Separator();

        if (ImGui::Button("Reset")) {
            tx = 0.6f;
            ty = 0.0f;
            tz = 0.0f;
        }

        ImGui::SliderFloat("translate x", &tx, -1.0f, 1.0f);
        ImGui::SliderFloat("translate y", &ty, -1.0f, 1.0f);
        ImGui::SliderFloat("translate z", &tz, -1.0f, 1.0f);

        t2 = Eigen::Vector3d(tx, ty, tz);

        V1w = V;
        V2w = V;

        for (auto& p : V1w) {
            p[0] += t1[0];
            p[1] += t1[1];
            p[2] += t1[2];
        }
        for (auto& p : V2w) {
            p[0] += t2[0];
            p[1] += t2[1];
            p[2] += t2[2];
        }

        box1->updateVertexPositions(V1w);
        box2->updateVertexPositions(V2w);
        geom_model.geometryObjects[id1].placement = pinocchio::SE3::Identity();
        geom_model.geometryObjects[id1].placement.translation() = t1;
        geom_model.geometryObjects[id2].placement = pinocchio::SE3::Identity();
        geom_model.geometryObjects[id2].placement.translation() = t2;
        pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data, q);

        // TODO 2: Compute collision and obtain the result of collision
        pinocchio::computeCollision(geom_model, geom_data, 0);
        in_collision = geom_data.collisionResults[0].isCollision();

        ImGui::Separator();
        if (in_collision) {
            ImGui::TextColored(ImVec4(1.f, 0.f, 0.f, 1.f), "In collision");
        } else {
            ImGui::TextColored(ImVec4(0.f, 1.f, 0.f, 1.f), "Collision-free");
        }
    };

    polyscope::show();
}