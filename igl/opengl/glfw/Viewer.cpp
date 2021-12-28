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
//#include <igl/get_seconds.h>
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
                isCCD(false)
            {
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


                //for (unsigned int i = 0; i<plugins.size(); ++i)
                //  if (plugins[i]->post_load())
                //    return true;

                if (mesh_file_name_string != "C:/Users/tal74/animation/animation3D/tutorial/data/sphere.obj") {
                    data().MyTranslateInSystem(data().GetRotation(), Eigen::RowVector3d(0, 0, 1.6));
                    data().kd_tree.init(data().V, data().F);
                    data().drawAxis(data().kd_tree.m_box);
                    data().SetCenterOfRotation(Eigen::RowVector3d(0, 0, -0.8));

                    int lastLinkidx = link_num;
                    tip = CalcParentsTrans(lastLinkidx) *
                        data(lastLinkidx).MakeTransd() *
                        Eigen::Vector4d(data(lastLinkidx).V.colwise().mean()[0], data(lastLinkidx).V.colwise().maxCoeff()[1], data(lastLinkidx).V.colwise().mean()[2], 1);

                    link_num++;
                }


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
                this->load_mesh_from_file("C:/Users/tal74/animation/animation3D/tutorial/data/zcylinder.obj");
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
                //if (visible)
                //    for (int i = 0; i < core_list.size(); i++)
                //        data_list.back().set_visible(true, core_list[i].id);
                //else
                //    data_list.back().is_visible = 0;
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


            Eigen::Matrix4d Viewer::CalcParentsTrans(int indx)
            {
                Eigen::Matrix4d prevTrans = Eigen::Matrix4d::Identity();

                for (int i = indx; parents[i] >= 0; i = parents[i])
                {
                    prevTrans = data_list[parents[i]].MakeTransd() * prevTrans;
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


            IGL_INLINE void Viewer::animateCCD() {
                Eigen::Vector3d ball = (data_list[0].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3),
                    E, R, RE, RD, cross;
                double dist = 0.0;

                for (int i = link_num; i > 0; --i) {
                    E = (CalcParentsTranslation(link_num) * data_list[link_num].MakeTransd() * Eigen::Vector4d(0, 0, link_length / 2, 1)).head(3);
                    dist = (E - ball).norm();

                    R = (CalcParentsTranslation(i) * data_list[i].MakeTransd() * Eigen::Vector4d(0, 0, -link_length / 2, 1)).head(3);
                    RE = E - R;
                    RD = ball - R;

                    double dot_product = RD.normalized().dot(RE.normalized());
                    double alpha = acos(dot_product > 1 ? 1 : dot_product < -1 ? -1 : dot_product);

                    cross = RE.cross(RD).normalized();
                    cross = CalcParentsInverseRotation(i) * cross;
                    data_list[i].MyRotate(cross, alpha / 30);
                }

                if (dist < 0.1) {
                    fixAxis();
                    isActive = false;
                }
            }

            void Viewer::animateFABRIK()
            {

                std::vector<Eigen::Vector3d> joints, new_joints;

                for (int i = 0; i <= link_num; i++) {
                    joints.push_back(calcJointPos(i + 1));
                }
                new_joints = joints;

                Eigen::Vector3d ball = (data_list[0].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3),
                    b = new_joints[0],
                    E, R, RE, RD, cross;

                double dist = (new_joints[new_joints.size() - 1] - ball).norm();

                //forward
                new_joints[new_joints.size() - 1] = ball;
                for (int i = link_num - 1; i > 0; --i) {
                    double r = (new_joints[i + 1] - new_joints[i]).norm(),
                        lambda = link_length / r;
                    new_joints[i] = (1 - lambda) * new_joints[i + 1] + lambda * new_joints[i];
                }

                //backword
                new_joints[0] = b;
                for (int i = 0; i < link_num; ++i) {
                    double r = (new_joints[i + 1] - new_joints[i]).norm(),
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

                if (dist < 0.1) {
                    fixAxis();
                    isActive = false;
                }

            }

            void Viewer::fixAxis() {
                for (int i = 1; i < data_list.size(); ++i) {
                    Eigen::Matrix3d rotation_matatrix = data_list[i].GetRotation();
                    Eigen::Vector3d euler_angle = rotation_matatrix.eulerAngles(2, 0, 2);
                    double rotZ = euler_angle[2];
                    data_list[i].MyRotate(Eigen::Vector3d(0, 0, 1), -rotZ);
                    if (i != link_num) {
                        data_list[i + 1].RotateInSystem(Eigen::Vector3d(0, 0, 1), rotZ);
                    }
                }
            }


        } // end namespace
    } // end namespace
}
