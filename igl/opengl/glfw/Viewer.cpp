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

  IGL_INLINE Viewer::Viewer():
    data_list(1),
    selected_data_index(0),
    next_data_id(1),
	isPicked(false),
	isActive(false),
    tip(Eigen::Vector4d(0, 0, 0, 0)),
    link_num(0),
    destination(Eigen::Vector3d(5,0,0)),
    ikAnimation(false)
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
    std::cout<<usage<<std::endl;
#endif
  }

  IGL_INLINE Viewer::~Viewer()
  {
  }

  IGL_INLINE Eigen::Matrix4d Viewer::MakeParentTrans(int mesh_id) {
      if (parents[mesh_id] == -1 || parents[mesh_id] == mesh_id)
          return Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix();
      Eigen::Matrix4d t = data_list[parents[mesh_id]].MakeTransd();
      Eigen::Matrix4d tp = MakeParentTrans(parents[mesh_id]);
      return MakeParentTrans(parents[mesh_id]) * data_list[parents[mesh_id]].MakeTransd();
  }

  IGL_INLINE Eigen::Matrix3d Viewer::GetParentsRotationInverse(int index) {

      Eigen::Matrix3d parentsInverse = data(index).GetRotation().inverse();
      int i = parents[index];
      while (i != -1)
      {
          parentsInverse = parentsInverse * data(i).GetRotation().inverse();
          i = parents[i];
      }

      return parentsInverse;
  }

  IGL_INLINE void Viewer::ik() {
      int iSphere = 0;
      int iFirstLink = 1;
      int iLastLink = data_list.size() - 1;

      Eigen::Vector3d d = data(iSphere).MakeTransd().block(0, 1, 3, 3).col(2);
      Eigen::Vector3d arm_base = data(iFirstLink).MakeTransd().block(0, 1, 3, 3).col(2);
      double link_size = 1.6;
      double num_of_links = link_num;
      if ((d - arm_base).norm() > link_size * num_of_links) {
          ikAnimation = false;
          std::cout << "cannot reach" << std::endl;
          return;
      }
      int i = iLastLink;
      while (i != -1 || i > iLastLink)
      {
          //iking = false;
          Eigen::Vector4d rCenter(data(i).GetCenterOfRotation()[0], data(i).GetCenterOfRotation()[1], data(i).GetCenterOfRotation()[2], 1);
          Eigen::Vector4d r4 = MakeParentTrans(i) * data(i).MakeTransd() * rCenter;
          Eigen::Vector3d r = r4.head<3>();

          Eigen::Vector4d eCenter(data(iLastLink).GetCenterOfRotation()[0], data(iLastLink).GetCenterOfRotation()[1] + 1.6, data(iLastLink).GetCenterOfRotation()[2], 1);
          Eigen::Vector4d e4 = MakeParentTrans(iLastLink) * data(iLastLink).MakeTransd() * eCenter;
          Eigen::Vector3d e = e4.head<3>();

          Eigen::Vector3d rd = d - r;
          Eigen::Vector3d re = e - r;

          double distance = (d - e).norm();

          if (distance < 0.1) {
              std::cout << "distance: " << distance << std::endl;
              ikAnimation = false;
              fin_rotate();
              return;
          }

          Eigen::Vector3d rotation_axis = GetParentsRotationInverse(i) * re.cross(rd).normalized();
          double dot = rd.normalized().dot(re.normalized());
          if (dot > 1)
              dot = 1;
          if (dot < -1)
              dot = -1;
          double angle = distance < 1 ? acosf(dot) : acosf(dot) / 10;
          //float angle = acosf(dot);

          //--------------bonus------------------------- 
          int parent_i = parents[i];
          if (parent_i != -1) {
              //update the vector re
              data(i).MyRotate(rotation_axis, angle);
              eCenter = Eigen::Vector4d(data(iLastLink).GetCenterOfRotation()[0], data(iLastLink).GetCenterOfRotation()[1] + 1.6, data(iLastLink).GetCenterOfRotation()[2], 1);
              e4 = MakeParentTrans(iLastLink) * data(iLastLink).MakeTransd() * eCenter;
              e = e4.head<3>();
              re = e - r;
              //get parent vector
              Eigen::Vector4d parent_rCenter(data(parent_i).GetCenterOfRotation()[0], data(parent_i).GetCenterOfRotation()[1], data(parent_i).GetCenterOfRotation()[2], 1);
              Eigen::Vector4d parent_r4 = MakeParentTrans(parent_i) * data(parent_i).MakeTransd() * parent_rCenter;
              Eigen::Vector3d parent_r = parent_r4.head<3>();
              Eigen::Vector3d parent_vec = parent_r - r;

              //find angle
              double parent_dot = parent_vec.normalized().dot(re.normalized());
              if (parent_dot > 1)
                  parent_dot = 1;
              if (parent_dot < -1)
                  parent_dot = -1;
              double parent_angle = acosf(parent_dot);
              double deg2rad = 0.017453292;

              //constrain
              double constrain = 30 * deg2rad;
              double fix = 0;
              if (parent_angle < constrain) {
                  fix = constrain - parent_angle;
              }
              data(i).MyRotate(rotation_axis, -angle); //rollback
              angle -= fix;
          }
          //--------------bonus-------------------------
          //float angle = acosf(dot);
          data(i).MyRotate(rotation_axis, angle);


          /*Eigen::Vector3f X(1, 0, 0);
          Eigen::Vector3f Y(0, 1, 0);
          Eigen::Matrix3f S;
          S << 0, -rotation_axis[1], rotation_axis[2],
              rotation_axis[1], 0, -rotation_axis[0],
              -rotation_axis[2], rotation_axis[0], 0;
          Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
          Eigen::Matrix3f R = I + sinf(angle) * S + (1 - cosf(angle)) * S * S;
          float r00 = R.row(0)[0];
          float r01 = R.row(0)[1];
          float r02 = R.row(0)[2];
          float r10 = R.row(1)[0];
          float r11 = R.row(1)[1];
          float r12 = R.row(1)[2];
          float r20 = R.row(2)[0];
          float r21 = R.row(2)[1];
          float r22 = R.row(2)[2];
          float angleY0;
          float angleX;
          float angleY1;
          //YXY
          if (r11 < 1) {
              if (r11 > - 1) {
                  angleX = acosf(r11);
                  angleY0 = atan2f(r01, r21);
                  angleY1 = atan2f(r10, -r12);
              }
              else { //r11 = -1
                  angleX = acosf(-1);
                  angleY0 = -atan2f(r02, r00);
                  angleY1 = 0;
              }
          }
          else {//r11 = 1
              angleX = 0;
              angleY0 = atan2f(r02, r00);
              angleY1 = 0;
          }
          data(i).MyRotate(Y, angleY0, false);
          data(i).MyRotate(X, angleX);
          data(i).MyRotate(Y, angleY1, false);*/

          i = parents[i];
      }
  }

  IGL_INLINE void Viewer::fin_rotate()
  {
      Eigen::Vector3d Z(0, 0, 1);
      Eigen::Matrix3d prev_z = Eigen::Matrix3d::Identity();
      Eigen::Matrix3d new_z = Eigen::Matrix3d::Identity();
      for (int i = 1; i <= data_list.size() - 1; i++) {

          Eigen::Matrix3d R = data(i).GetRotation();

          double r00 = R.row(0)[0];  // ux
          double r01 = R.row(0)[1];  // uy
          double r02 = R.row(0)[2];  // uz
          double r10 = R.row(1)[0];  // vx
          double r11 = R.row(1)[1];  // vy
          double r12 = R.row(1)[2];  // vz
          double r20 = R.row(2)[0];  // wx
          double r21 = R.row(2)[1];  // wy
          double r22 = R.row(2)[2];  // wz

          double angleZ0;
          double angleY;
          double angleZ1;
          // y -> z, x -> y , z -> x
          //ZYZ
          if (r12 < 1) {
              if (r12 > -1) {
                  angleY = acosf(r12);
                  angleZ0 = atan2f(r02, r22);
                  angleZ1 = atan2f(r11, -r10);
              }
              else {
                  angleY = acosf(-1);
                  angleZ0 = -atan2f(r00, r01);
                  angleZ1 = 0;
              }
          }
          else {
              angleY = 0;
              angleZ0 = atan2f(r00, r01);
              angleZ1 = 0;
          }

          data(i).MyRotate(Z, -angleZ1);
          if (i != data_list.size() - 1)
              data(i + 1).MyRotate(Z, angleZ1);
      }
  }



  IGL_INLINE bool Viewer::load_mesh_from_file(
      const std::string & mesh_file_name_string)
  {

    // Create new data slot and set to selected
    if(!(data().F.rows() == 0  && data().V.rows() == 0))
    {
      append_mesh();
    }
    data().clear();

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }

    std::string extension = mesh_file_name_string.substr(last_dot+1);

    if (extension == "off" || extension =="OFF")
    {
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;
      if (!igl::readOFF(mesh_file_name_string, V, F))
        return false;
      data().set_mesh(V,F);
    }
    else if (extension == "obj" || extension =="OBJ")
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

      data().set_mesh(V,F);
      if (UV_V.rows() > 0)
      {
          data().set_uv(UV_V, UV_F);
      }

    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }

    data().compute_normals();
    data().uniform_colors(Eigen::Vector3d(51.0/255.0,43.0/255.0,33.3/255.0),
                   Eigen::Vector3d(255.0/255.0,228.0/255.0,58.0/255.0),
                   Eigen::Vector3d(255.0/255.0,235.0/255.0,80.0/255.0));

    // Alec: why?
    if (data().V_uv.rows() == 0)
    {
      data().grid_texture();
    }


    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->post_load())
    //    return true;

    if(mesh_file_name_string != "C:/Users/tal74/animation/animation3D/tutorial/data/sphere.obj") {
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
      const std::string & mesh_file_name_string)
  {
    // first try to load it with a plugin
    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->save(mesh_file_name_string))
    //    return true;

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      // No file type determined
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }
    std::string extension = mesh_file_name_string.substr(last_dot+1);
    if (extension == "off" || extension =="OFF")
    {
      return igl::writeOFF(
        mesh_file_name_string,data().V,data().F);
    }
    else if (extension == "obj" || extension =="OBJ")
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
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }
    return true;
  }
 
  IGL_INLINE bool Viewer::load_scene()
  {
    std::string fname = igl::file_dialog_open();
    if(fname.length() == 0)
      return false;
    return load_scene(fname);
  }

  IGL_INLINE bool Viewer::load_scene(std::string fname)
  {
   // igl::deserialize(core(),"Core",fname.c_str());
    igl::deserialize(data(),"Data",fname.c_str());
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
    igl::serialize(data(),"Data",fname.c_str());

    return true;
  }

  IGL_INLINE void Viewer::open_dialog_load_mesh()
  {
    this->load_mesh_from_file("C:/Users/tal74/animation/animation3D/tutorial/data/zcylinder.obj");
  }

  IGL_INLINE void Viewer::open_dialog_save_mesh()
  {
    std::string fname = igl::file_dialog_save();

    if(fname.length() == 0)
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
    selected_data_index = data_list.size()-1;
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
    if(data_list.size() == 1)
    {
      // Cannot remove last mesh
      return false;
    }
    data_list[index].meshgl.free();
    data_list.erase(data_list.begin() + index);
    if(selected_data_index >= index && selected_data_index > 0)
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
		  //std::cout << "parent matrix:\n" << scn->data_list[scn->parents[i]].MakeTrans() << std::endl;
		  prevTrans = data_list[parents[i]].MakeTransd() * prevTrans;
	  }

	  return prevTrans;
  }

} // end namespace
} // end namespace
}
