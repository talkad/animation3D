// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"

//#include <chrono>
#include <thread>

#include <Eigen/LU>


#include <cmath>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cassert>

#include <igl/project.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>
#include <igl/get_seconds.h>
#include <GLFW/glfw3.h>
#include <igl/dqs.h>
#include <Windows.h>
#include <MMSystem.h>
#pragma comment(lib, "winmm.lib")


// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;


namespace igl
{
    namespace opengl
    {
        namespace glfw
        {

            void Viewer::Init(const std::string config)
            {


            }

            IGL_INLINE Viewer::Viewer() :
                data_list(1),
                selected_data_index(0),
                next_data_id(1),
                isPicked(false),
                isActive(false),
                tip(Eigen::Vector4d(0, 0, 0, 0)),
                link_num(0),
                destination(Eigen::Vector3d(5, 0, 0)),
                ikAnimation(false),
                link_length(1.6),
                snake_length(1.6),
                snake_tail_start(-0.8),
                current_picked(-1),
                delta(0.1),
                snake_size(1),  // currently the head will be the circle
                snake_view(false),
                prev_tic(0),
                TTL(25),
                level(1),
                score(0),
                start_time(0),
                target2_creation(2),
                joints_num(16),
                scale(1),
                direction(' '),
                previous_direction('r'),
                isGameOver(false),
                start(true),
                isResume(false),
                isGameStarted(false),
                isPaused(false),
                timer(0),
                pause_time(0),
                resume_time(0),
                paused_time(0),
                isFog(true),
                isFP(false),
                isLevelUp(false),
                update_camera_rotation(false),
                keyPressed(' '),
                level1_obj_amount(0),
                creation_gap(0)
            {
                jointBoxes.resize(joints_num);
                vT.resize(joints_num + 1);
                vQ.resize(joints_num + 1);

                data_list.front().id = 0;

                // Temporary variables initialization
               // down = false;
              //  hack_never_moved = true;
                scroll_position = 0.0f;

                // Per face
                data().set_face_based(false);


#ifndef IGL_VIEWER_VIEWER_QUIET
                const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  [,]     Toggle between cameras
  1,2     Toggle between models
  ;       Toggle vertex labels
  :       Toggle face labels)"
                );
                std::cout << usage << std::endl;
#endif
            }

            IGL_INLINE Viewer::~Viewer()
            {
            }

            IGL_INLINE bool Viewer::load_mesh_from_file(
                const std::string& mesh_file_name_string)
            {

                // Create new data slot and set to selected
                if (!(data().F.rows() == 0 && data().V.rows() == 0))
                {
                    append_mesh();
                }
                data().clear();

                size_t last_dot = mesh_file_name_string.rfind('.');
                if (last_dot == std::string::npos)
                {
                    std::cerr << "Error: No file extension found in " <<
                        mesh_file_name_string << std::endl;
                    return false;
                }

                std::string extension = mesh_file_name_string.substr(last_dot + 1);

                if (extension == "off" || extension == "OFF")
                {
                    Eigen::MatrixXd V;
                    Eigen::MatrixXi F;
                    if (!igl::readOFF(mesh_file_name_string, V, F))
                        return false;
                    data().set_mesh(V, F);
                }
                else if (extension == "obj" || extension == "OBJ")
                {
                    Eigen::MatrixXd corner_normals;
                    Eigen::MatrixXi fNormIndices;

                    Eigen::MatrixXd UV_V;
                    Eigen::MatrixXi UV_F;
                    Eigen::MatrixXd V;
                    Eigen::MatrixXi F;

                    if (!(
                        igl::readOBJ(
                            mesh_file_name_string,
                            V, UV_V, corner_normals, F, UV_F, fNormIndices)))
                    {
                        return false;
                    }

                    data().set_mesh(V, F);
                    if (UV_V.rows() > 0)
                    {
                        data().set_uv(UV_V, UV_F);
                    }

                }
                else
                {
                    // unrecognized file type
                    printf("Error: %s is not a recognized file type.\n", extension.c_str());
                    return false;
                }

                data().compute_normals();
                data().uniform_colors(Eigen::Vector3d(51.0 / 255.0, 43.0 / 255.0, 33.3 / 255.0),
                    Eigen::Vector3d(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0),
                    Eigen::Vector3d(255.0 / 255.0, 235.0 / 255.0, 80.0 / 255.0));

                // Alec: why?
                if (data().V_uv.rows() == 0)
                {
                    data().grid_texture();
                }

                data().init(); // initiate object fields

              

                return true;
            }

            IGL_INLINE bool Viewer::save_mesh_to_file(
                const std::string& mesh_file_name_string)
            {
                // first try to load it with a plugin
                //for (unsigned int i = 0; i<plugins.size(); ++i)
                //  if (plugins[i]->save(mesh_file_name_string))
                //    return true;

                size_t last_dot = mesh_file_name_string.rfind('.');
                if (last_dot == std::string::npos)
                {
                    // No file type determined
                    std::cerr << "Error: No file extension found in " <<
                        mesh_file_name_string << std::endl;
                    return false;
                }
                std::string extension = mesh_file_name_string.substr(last_dot + 1);
                if (extension == "off" || extension == "OFF")
                {
                    return igl::writeOFF(
                        mesh_file_name_string, data().V, data().F);
                }
                else if (extension == "obj" || extension == "OBJ")
                {
                    Eigen::MatrixXd corner_normals;
                    Eigen::MatrixXi fNormIndices;

                    Eigen::MatrixXd UV_V;
                    Eigen::MatrixXi UV_F;

                    return igl::writeOBJ(mesh_file_name_string,
                        data().V,
                        data().F,
                        corner_normals, fNormIndices, UV_V, UV_F);
                }
                else
                {
                    // unrecognized file type
                    printf("Error: %s is not a recognized file type.\n", extension.c_str());
                    return false;
                }
                return true;
            }

            IGL_INLINE bool Viewer::load_scene()
            {
                std::string fname = igl::file_dialog_open();
                if (fname.length() == 0)
                    return false;
                return load_scene(fname);
            }

            IGL_INLINE bool Viewer::load_scene(std::string fname)
            {
                // igl::deserialize(core(),"Core",fname.c_str());
                igl::deserialize(data(), "Data", fname.c_str());
                return true;
            }

            IGL_INLINE bool Viewer::save_scene()
            {
                std::string fname = igl::file_dialog_save();
                if (fname.length() == 0)
                    return false;
                return save_scene(fname);
            }

            IGL_INLINE bool Viewer::save_scene(std::string fname)
            {
                //igl::serialize(core(),"Core",fname.c_str(),true);
                igl::serialize(data(), "Data", fname.c_str());

                return true;
            }

            IGL_INLINE void Viewer::open_dialog_load_mesh()
            {
                std::string fname = igl::file_dialog_open();

                if (fname.length() == 0)
                    return;

                this->load_mesh_from_file(fname.c_str());
            }

            IGL_INLINE void Viewer::open_dialog_save_mesh()
            {
                std::string fname = igl::file_dialog_save();

                if (fname.length() == 0)
                    return;

                this->save_mesh_to_file(fname.c_str());
            }

            IGL_INLINE ViewerData& Viewer::data(int mesh_id /*= -1*/)
            {
                assert(!data_list.empty() && "data_list should never be empty");
                int index;
                if (mesh_id == -1)
                    index = selected_data_index;
                else
                    index = mesh_index(mesh_id);

                assert((index >= 0 && index < data_list.size()) &&
                    "selected_data_index or mesh_id should be in bounds");
                return data_list[index];
            }

            IGL_INLINE const ViewerData& Viewer::data(int mesh_id /*= -1*/) const
            {
                assert(!data_list.empty() && "data_list should never be empty");
                int index;
                if (mesh_id == -1)
                    index = selected_data_index;
                else
                    index = mesh_index(mesh_id);

                assert((index >= 0 && index < data_list.size()) &&
                    "selected_data_index or mesh_id should be in bounds");
                return data_list[index];
            }

            IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/)
            {
                assert(data_list.size() >= 1);

                data_list.emplace_back();
                selected_data_index = data_list.size() - 1;
                data_list.back().id = next_data_id++;

                return data_list.back().id;
            }

            IGL_INLINE bool Viewer::erase_mesh(const size_t index)
            {
                assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
                assert(data_list.size() >= 1);
                if (data_list.size() == 1)
                {
                    // Cannot remove last mesh
                    return false;
                }
                data_list[index].meshgl.free();
                data_list.erase(data_list.begin() + index);
                if (selected_data_index >= index && selected_data_index > 0)
                {
                    selected_data_index--;
                }

                return true;
            }

            IGL_INLINE size_t Viewer::mesh_index(const int id) const {
                for (size_t i = 0; i < data_list.size(); ++i)
                {
                    if (data_list[i].id == id)
                        return i;
                }
                return 0;
            }

            IGL_INLINE void Viewer::clean_data_list() {
                // attempt to use erase_mesh ended up with failure

                if (!isGameStarted || isPaused || isGameOver)
                    return;

                float tic = static_cast<float>(glfwGetTime());
                for (auto& mesh : data_list) {
                    if (mesh.type != NONE && !mesh.isTerminated && tic - mesh.creation_time > TTL) {
                        mesh.is_visible = false;
                        mesh.update_movement_type(NONE);
                        mesh.isTerminated = true;
                    }
                }
                
            }

            IGL_INLINE bool Viewer::boxes_collide(Eigen::AlignedBox<double, 3>& firstbox, Eigen::AlignedBox<double, 3>& secondbox, int i, int j) {
                double a0 = firstbox.sizes().x() / 2, a1 = firstbox.sizes().y() / 2, a2 = firstbox.sizes().z() / 2,
                    b0 = secondbox.sizes().x() / 2, b1 = secondbox.sizes().y() / 2, b2 = secondbox.sizes().z() / 2,
                    R0, R1, R;
                Eigen::Matrix3d A, B, C;
                Eigen::Vector3d D, C0, C1;
                Eigen::RowVector3d  A0 = split_snake[j].GetRotation().col(0),
                                    A1 = split_snake[j].GetRotation().col(1),
                                    A2 = split_snake[j].GetRotation().col(2),
                                    B0 = data_list[i].GetRotation().col(0),
                                    B1 = data_list[i].GetRotation().col(1),
                                    B2 = data_list[i].GetRotation().col(2);
                A << split_snake[j].GetRotation();
                B << data_list[i].GetRotation();
                C = A.transpose() * B;

                Eigen::Vector4d C0_4cord = split_snake[j].MakeTransScaled() * Eigen::Vector4d(firstbox.center().x(), firstbox.center().y(), firstbox.center().z(), 1);
                Eigen::Vector4d C1_4cord = data_list[i].MakeTransScaled() * Eigen::Vector4d(secondbox.center().x(), secondbox.center().y(), secondbox.center().z(), 1);

                C0 = C0_4cord.head(3);
                C1 = C1_4cord.head(3);

                D = C1 - C0;

                //Table case 1
                R0 = a0;
                R1 = (b0 * abs(C(0, 0))) + (b1 * abs(C(0, 1))) + (b2 * abs(C(0, 2)));
                R = abs(A0.dot(D));

                if (R > R0 + R1) return false;

                //Table case 2
                R0 = a1;
                R1 = (b0 * abs(C(1, 0))) + (b1 * abs(C(1, 1))) + (b2 * abs(C(1, 2)));
                R = abs(A1.dot(D));

                if (R > R0 + R1) return false;

                //Table case 3
                R0 = a2;
                R1 = (b0 * abs(C(2, 0))) + (b1 * abs(C(2, 1))) + (b2 * abs(C(2, 2)));
                R = abs(A2.dot(D));

                if (R > R0 + R1) return false;

                //Table case 4
                R0 = (a0 * abs(C(0, 0))) + (a1 * abs(C(1, 0))) + (a2 * abs(C(2, 0)));
                R1 = b0;
                R = abs(B0.dot(D));

                if (R > R0 + R1) return false;

                //Table case 5
                R0 = (a0 * abs(C(0, 1))) + (a1 * abs(C(1, 1))) + (a2 * abs(C(2, 1)));
                R1 = b1;
                R = abs(B1.dot(D));

                if (R > R0 + R1) return false;

                //Table case 6
                R0 = (a0 * abs(C(0, 2))) + (a1 * abs(C(1, 2))) + (a2 * abs(C(2, 2)));
                R1 = b2;
                R = abs(B2.dot(D));

                if (R > R0 + R1) return false;

                //Table case 7
                R0 = (a1 * abs(C(2, 0))) + (a2 * abs(C(1, 0)));
                R1 = (b1 * abs(C(0, 2))) + (b2 * abs(C(0, 1)));
                R = abs((C(1, 0) * A2).dot(D) - (C(2, 0) * A1).dot(D));

                if (R > R0 + R1) return false;

                //Table case 8
                R0 = (a1 * abs(C(2, 1))) + (a2 * abs(C(1, 1)));
                R1 = (b0 * abs(C(0, 2))) + (b2 * abs(C(0, 0)));
                R = abs((C(1, 1) * A2).dot(D) - (C(2, 1) * A1).dot(D));

                if (R > R0 + R1) return false;

                //Table case 9
                R0 = (a1 * abs(C(2, 2))) + (a2 * abs(C(1, 2)));
                R1 = (b0 * abs(C(0, 1))) + (b1 * abs(C(0, 0)));
                R = abs((C(1, 2) * A2).dot(D) - (C(2, 2) * A1).dot(D));

                if (R > R0 + R1) return false;

                //Table case 10
                R0 = (a0 * abs(C(2, 0))) + (a2 * abs(C(0, 0)));
                R1 = (b1 * abs(C(1, 2))) + (b2 * abs(C(1, 1)));
                R = abs((C(2, 0) * A0).dot(D) - (C(0, 0) * A2).dot(D));

                if (R > R0 + R1) return false;

                //Table case 11
                R0 = (a0 * abs(C(2, 1))) + (a2 * abs(C(0, 1)));
                R1 = (b0 * abs(C(1, 2))) + (b2 * abs(C(1, 0)));
                R = abs((C(2, 1) * A0).dot(D) - (C(0, 1) * A2).dot(D));

                if (R > R0 + R1) return false;

                //Table case 12
                R0 = (a0 * abs(C(2, 2))) + (a2 * abs(C(0, 2)));
                R1 = (b0 * abs(C(1, 1))) + (b1 * abs(C(1, 0)));
                R = abs((C(2, 2) * A0).dot(D) - (C(0, 2) * A2).dot(D));

                if (R > R0 + R1) return false;

                //Table case 13
                R0 = (a0 * abs(C(1, 0))) + (a1 * abs(C(0, 0)));
                R1 = (b1 * abs(C(2, 2))) + (b2 * abs(C(2, 1)));
                R = abs((C(0, 0) * A1).dot(D) - (C(1, 0) * A0).dot(D));

                if (R > R0 + R1) return false;

                //Table case 14
                R0 = (a0 * abs(C(1, 1))) + (a1 * abs(C(0, 1)));
                R1 = (b0 * abs(C(2, 2))) + (b2 * abs(C(2, 0)));
                R = abs((C(0, 1) * A1).dot(D) - (C(1, 1) * A0).dot(D));

                if (R > R0 + R1) return false;

                //Table case 15
                R0 = (a0 * abs(C(1, 2))) + (a1 * abs(C(0, 2)));
                R1 = (b0 * abs(C(2, 1))) + (b1 * abs(C(2, 0)));
                R = abs((C(0, 2) * A1).dot(D) - (C(1, 2) * A0).dot(D));

                if (R > R0 + R1) return false;

                return true;
            }

            IGL_INLINE bool Viewer::treeNodesCollide(AABB<Eigen::MatrixXd, 3>& firstObjNode, Eigen::AlignedBox<double, 3>& secondObjNode, int i, int j) {
                if (boxes_collide(firstObjNode.m_box, secondObjNode, i, j)) {
                    if (firstObjNode.is_leaf()) 
                        return true;                    
                    else 
                         return treeNodesCollide(*firstObjNode.m_left, secondObjNode, i, j) ||
                                treeNodesCollide(*firstObjNode.m_right, secondObjNode, i, j);
                }
                return false;
            }

            // check if two object in data_list are collided
            // assume there are exactly two objects
            IGL_INLINE void Viewer::check_collision() {
                for(int i = 1; i < data_list.size(); ++i)
                    if (!data_list[i].is_collided && data_list[i].is_visible) {
                        for (int j = 0; j < jointBoxes.size(); ++j) {
                            if (treeNodesCollide(data_list[i].kd_tree, jointBoxes[j], i, j)) {
                                add_score(data_list[i].type);
                                data_list[i].is_visible = false;
                                data_list[i].is_collided = true;
                            }
                        }
                        
                    }
            }
  
            IGL_INLINE void Viewer::move_targets()
            {
                for (auto& data : data_list) 
                    if (data.type != NONE)
                        data.move();   
            }

            IGL_INLINE void Viewer::generate_target()
            {
                if (!isGameStarted || isPaused || isGameOver)
                    return;

                if (level == 1 && level1_obj_amount > 3)
                    return;

                float tic = static_cast<float>(glfwGetTime());

                if (tic - prev_tic > creation_gap) {
                    prev_tic = tic;

                    std::this_thread::sleep_for(std::chrono::microseconds(5));

                    load_mesh_from_file("C:/Users/pijon/OneDrive/Desktop/animation3D/tutorial/data/sphere.obj");
                    if (data_list.size() > parents.size())
                    {
                        parents.push_back(-1);
                        data_list.back().set_visible(false, 1);
                        data_list.back().set_visible(true, 2);
                        data_list.back().show_faces = 3;
                    }

                    if (level == 1) {
                        data().update_movement_type(BASIC);
                    }
                    else if (target2_creation == 0) { // generate different targets according to level
                        data().update_movement_type(BEZIER);
                        target2_creation = 3;
                    }
                    else {
                        double target_proba = (double)(rand() % 10) / 10;
                        
                        target_proba < p ? data().update_movement_type(BASIC):
                                           data().update_movement_type(BOUNCY);

                        target2_creation--;
                    }
                    
                    data().type == BEZIER ? data().set_colors(Eigen::RowVector3d(0, 0, 1)):
                    data().type == BOUNCY ? data().set_colors(Eigen::RowVector3d(1, 0, 0)):
                                            data().set_colors(Eigen::RowVector3d(0, 1, 0));

                    data().initiate_speed(level1_obj_amount);
                    level1_obj_amount++;                        
                }
            }


            // start a new level
            IGL_INLINE void Viewer::start_level() {

                start_time = static_cast<int>(glfwGetTime());
                prev_tic = static_cast<int>(glfwGetTime());
                paused_time = 0;

                p = 1.0 / level + 0.33;
            }

            IGL_INLINE void Viewer::check_level_up() {
                if (score >= 40 * level) {
                    level++;
                    isLevelUp = true;
                    isActive = false;
                    score = 0;
                    timer = 0;

                    creation_gap = 2;

                    PlaySound(TEXT("C:/Users/pijon/OneDrive/Desktop/animation3D/tutorial/sounds/nextLevel.wav"), NULL, SND_NODEFAULT | SND_ASYNC);
                }
            }

            IGL_INLINE void Viewer::add_score(int type) {
                type == BASIC  ? score += 10 :
                type == BOUNCY ? score += 20 :
                type == BEZIER ? score += 30 :
                                 score += 0;
                    // activate special abilities

                PlaySound(TEXT("C:/Users/pijon/OneDrive/Desktop/animation3D/tutorial/sounds/collision.wav"), NULL, SND_NODEFAULT | SND_ASYNC);
            }

            IGL_INLINE void Viewer::update_timer() {
                if(!isLevelUp){
                    int offset = static_cast<int>(glfwGetTime()) - start_time;
                    timer = (level * 30) - offset + paused_time;
                }

                if (timer == 0 && !isLevelUp) {
                    isGameOver = true;
                    PlaySound(TEXT("C:/Users/pijon/OneDrive/Desktop/animation3D/tutorial/sounds/gameOver.wav"), NULL, SND_NODEFAULT | SND_ASYNC);
                }
            }

            IGL_INLINE Eigen::VectorXd Viewer::create_weight_vec(double w1, int idx_w1, double w2, int idx_w2)
            {
                Eigen::VectorXd Wi;
                Wi.resize(joints_num + 1);

                for (int i = 0; i < Wi.size(); ++i)
                {
                    i == idx_w1 ? Wi[idx_w1] = w1 :
                    i == idx_w2 ? Wi[idx_w2] = w2 :
                                  Wi[i] = 0;
                }
                return Wi;
            }

            IGL_INLINE void Viewer::weights_calc()
            {
                Eigen::MatrixXd V = data_list[0].V;
                int vertexNum = V.rows();
                W.resize(vertexNum, joints_num + 1);

                double z, w1, w2, lBound, uBound;
                for (int i = 0; i < vertexNum; ++i) {
                    z = 10 * V.row(i).z();
                    lBound = floor(z);
                    uBound = ceil(z);

                    w1 = abs(z - uBound);
                    w2 = 1 - w1;

                    W.row(i) = create_weight_vec(w1, lBound + 8, w2, uBound + 8);
                }
            }

            IGL_INLINE void Viewer::next_vertices_position()
            {
                vT[0] = skeleton[0];

                for (int i = 0; i < joints_num; ++i) {
                    vT[i + 1] = skeleton[i + 1];
                    vT[i] += (vT[i + 1] - vT[i]) / 6;
                }

                vT[joints_num] += position_offset;
            }

            IGL_INLINE void Viewer::add_weights() {
                double distance;
                Eigen::MatrixXd V = data_list[0].V;
                int vertexNum = V.rows();
                W.resize(vertexNum, 17);

                for (int i = 0; i < vertexNum; ++i) {
                    double relative_distance = relative_to_segments_distance(i);
                    for (int j = 0; j < skeleton.size(); ++j) {
                        distance = abs(skeleton[j].z() - V.row(i).z());
                        W(i, j) = pow((1 / distance), 4) / relative_distance;
                    }
                    W.row(i).normalized();
                }
            }

            IGL_INLINE double Viewer::relative_to_segments_distance(int i) {
                double sum = 0, distance;

                for (int j = 0; j < skeleton.size(); ++j) {
                    distance = abs(skeleton[j].z() - V.row(i).z());
                    if (distance <= delta)
                        sum += pow((1 / distance), 4);
                }
                return sum;
            }

            IGL_INLINE void Viewer::createJointBoxes() {
                double epsilon = 0.4;

                for (int i = 1; i < joints_num + 1; ++i)
                {
                    Eigen::Vector3d pos = skeleton[i - 1];
                    Eigen::Vector3d m = pos + Eigen::Vector3d(-epsilon, -epsilon, -epsilon);
                    Eigen::Vector3d M = pos + Eigen::Vector3d(epsilon, epsilon, epsilon);
                    Eigen::AlignedBox<double, 3> boxforcurrJoint(m, M);
                    jointBoxes[i - 1] = boxforcurrJoint;
                }
            }

            IGL_INLINE void Viewer::move_snake() {
                double snake_speed = 0.15;

                if (!isGameStarted || isPaused || isGameOver) {
                    position_offset = Eigen::Vector3d::Zero();
                    return;
                }

                if (direction != ' ')
                {
                    switch (direction) {
                    case 'l':
                        position_offset = Eigen::Vector3d(0, 0, -snake_speed);
                        break;
                    case 'r':
                        position_offset = Eigen::Vector3d(0, 0, snake_speed);
                        break;
                    case 'u':
                        position_offset = Eigen::Vector3d(0, snake_speed, 0);
                        break;
                    case 'd':
                        position_offset = Eigen::Vector3d(0, -snake_speed, 0);
                        break;
                    default:
                        break;
                    }

                    next_vertices_position();
                    igl::dqs(V, W, vQ, vT, U);
                    data_list[0].set_vertices(U);

                    for (int i = 0; i < split_snake.size(); ++i){
                        Eigen::Vector3d pos_offset = vT[i] - skeleton[i];
                        Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(pos_offset.reverse(), skeleton[i].reverse());

                        split_snake[i].MyTranslate(pos_offset.reverse(), true);
                        split_snake[i].MyRotate(quat);
                    }

                    for (int i = 0; i < skeleton.size(); ++i)
                        skeleton[i] = vT[i];
                    
                    check_collision();
                }
            }

            IGL_INLINE Eigen::Matrix4d Viewer::CalcParentsTrans(int indx)
            {
                Eigen::Matrix4d prevTrans = Eigen::Matrix4d::Identity();

                for (int i = indx; parents[i] >= 0; i = parents[i])
                {
                    prevTrans = data_list[parents[i]].MakeTransd() * prevTrans;
                }

                return prevTrans;
            }

            IGL_INLINE Eigen::Matrix4d Viewer::CalcSnakeJointsTrans()
            {
                Eigen::Matrix4d prevTrans = Eigen::Matrix4d::Identity();

                for (int i = 15; i >= 0; --i)
                {
                    prevTrans = split_snake[i].MakeTransd() * prevTrans;
                }

                return prevTrans;
            }



            IGL_INLINE Eigen::Matrix4d Viewer::CalcParentsTranslation(int index) {
                return (index <= 1) ? Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix() :
                    CalcParentsTranslation(index - 1) * data_list[index - 1].MakeTransd();
            }


            IGL_INLINE Eigen::Vector3d Viewer::calcJointPos(int jointPos)
            {
                return (jointPos == data_list.size())? (CalcParentsTranslation(link_num) * data_list[link_num].MakeTransd() * Eigen::Vector4d(0, 0, link_length / 2, 1)).head(3) : 
                                                        (CalcParentsTranslation(jointPos) * data_list[jointPos].MakeTransd() * Eigen::Vector4d(0, 0, -link_length / 2, 1)).head(3);
            }

            IGL_INLINE Eigen::Matrix3d Viewer::CalcParentsInverseRotation(int index) {
                Eigen::Matrix3d rot = data(index).GetRotation().inverse();

                for (int i = index - 1; i > 0; --i)
                    rot *= data(i).GetRotation().inverse();

                return rot;
            }

            void Viewer::animateFABRIK()
            {
                std::vector<Eigen::Vector3d> joints, new_joints;

                for (int i = 0; i <= link_num; i++)
                    joints.push_back(calcJointPos(i + 1));
                
                new_joints = joints;

                Eigen::Vector3d ball = (data_list[0].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3),
                    b = new_joints[0],
                    E, R, RE, RD, cross;

                double dist = (new_joints[new_joints.size() - 1] - ball).norm(),
                       r, lambda;

                //forward
                new_joints[new_joints.size() - 1] = ball;
                for (int i = link_num - 1; i > 0; --i) {
                    r = (new_joints[i + 1] - new_joints[i]).norm(),
                    lambda = link_length / r;
                    new_joints[i] = (1 - lambda) * new_joints[i + 1] + lambda * new_joints[i];
                }

                //backword
                new_joints[0] = b;
                for (int i = 0; i < link_num; ++i) {
                    r = (new_joints[i + 1] - new_joints[i]).norm(),
                    lambda = link_length / r;
                    new_joints[i + 1] = (1 - lambda) * new_joints[i] + lambda * new_joints[i + 1];
                }

                // motion
                for (int i = 0; i < new_joints.size() - 1; ++i) {
                    RE = joints[i + 1] - joints[i];
                    RD = new_joints[i + 1] - joints[i];
                    double dot_product = RD.normalized().dot(RE.normalized());
                    double alpha = acos(dot_product > 1 ? 1 : dot_product < -1 ? -1 : dot_product);

                    cross = RE.cross(RD).normalized();
                    cross = CalcParentsInverseRotation(i + 1) * cross;
                    data_list[i + 1].MyRotate(cross, alpha / 30);
                }

                if (dist < delta) {
                    fixAxis();
                    isActive = false;
                }

            }


            void Viewer::fixAxis() {
                for (int i = 1; i < data_list.size(); ++i) {
                    Eigen::Matrix3d rotation_matrix = data_list[i].GetRotation();
                    Eigen::Vector3d euler_angle = rotation_matrix.eulerAngles(2, 0, 2);
                    double rotZ = euler_angle[2];
                    data_list[i].MyRotate(Eigen::Vector3d::UnitZ(), -rotZ);
                    if (i != link_num) 
                        data_list[i + 1].RotateInSystem(Eigen::Vector3d::UnitZ(), rotZ);
                }
            }
        } // end namespace
    } // end namespace
}