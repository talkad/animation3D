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
    isCollide(false)
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

  IGL_INLINE bool Viewer::boxes_collide(Eigen::AlignedBox<double, 3>& firstbox, Eigen::AlignedBox<double, 3>& secondbox) {
      double a0 = firstbox.sizes()[0] / 2, a1 = firstbox.sizes()[1] / 2, a2 = firstbox.sizes()[2] / 2,
             b0 = secondbox.sizes()[0] / 2, b1 = secondbox.sizes()[1] / 2, b2 = secondbox.sizes()[2] / 2,
             R0, R1, R;
      Eigen::Matrix3d A, B, C;
      Eigen::Vector3d D, C0, C1;
      Eigen::RowVector3d A0 = data_list[0].GetRotation() * Eigen::Vector3d(1, 0, 0),
                         A1 = data_list[0].GetRotation() * Eigen::Vector3d(0, 1, 0),
                         A2 = data_list[0].GetRotation() * Eigen::Vector3d(0, 0, 1),
                         B0 = data_list[1].GetRotation() * Eigen::Vector3d(1, 0, 0),
                         B1 = data_list[1].GetRotation() * Eigen::Vector3d(0, 1, 0),
                         B2 = data_list[1].GetRotation() * Eigen::Vector3d(0, 0, 1);
      A << Eigen::RowVector3d(A0[0], A1[0], A2[0]), Eigen::RowVector3d(A0[1], A1[1], A2[1]), Eigen::RowVector3d(A0[2], A1[2], A2[2]);
      B << Eigen::RowVector3d(B0[0], B1[0], B2[0]), Eigen::RowVector3d(B0[1], B1[1], B2[1]), Eigen::RowVector3d(B0[2], B1[2], B2[2]);
      C = A.transpose() * B;

      Eigen::Vector4f C0_4cord = data_list[0].MakeTransScale() * Eigen::Vector4f(firstbox.center()[0], firstbox.center()[1], firstbox.center()[2], 1);
      Eigen::Vector4f C1_4cord = data_list[1].MakeTransScale() * Eigen::Vector4f(secondbox.center()[0], secondbox.center()[1], secondbox.center()[2], 1);
     
      C0 = Eigen::Vector3d(C0_4cord[0], C0_4cord[1], C0_4cord[2]);
      C1 = Eigen::Vector3d(C1_4cord[0], C1_4cord[1], C1_4cord[2]);

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

  IGL_INLINE bool Viewer::treeNodesCollide(AABB<Eigen::MatrixXd, 3>& firstObjNode, AABB<Eigen::MatrixXd, 3>& secondObjNode) {
      if (boxes_collide(firstObjNode.m_box, secondObjNode.m_box)) {
          if (firstObjNode.is_leaf() && secondObjNode.is_leaf()) {
              data_list[0].drawAlignedBox(firstObjNode.m_box, Eigen::RowVector3d(1, 0, 0));
              data_list[1].drawAlignedBox(secondObjNode.m_box, Eigen::RowVector3d(1, 0, 0));
              return true;
          }
          else {
              if (firstObjNode.is_leaf()) {
                  if (secondObjNode.m_left)
                      return treeNodesCollide(firstObjNode, *secondObjNode.m_left);
                  if (secondObjNode.m_right)
                      return treeNodesCollide(firstObjNode, *secondObjNode.m_right);
              }
              else if (secondObjNode.is_leaf()) {
                  if (firstObjNode.m_left)
                      return treeNodesCollide(*firstObjNode.m_left, secondObjNode);
                  if (firstObjNode.m_right)
                      return treeNodesCollide(*firstObjNode.m_right, secondObjNode);
              }
              else
                  return treeNodesCollide(*firstObjNode.m_left, *secondObjNode.m_left) ||
                         treeNodesCollide(*firstObjNode.m_left, *secondObjNode.m_right) ||
                         treeNodesCollide(*firstObjNode.m_right, *secondObjNode.m_left) ||
                         treeNodesCollide(*firstObjNode.m_right, *secondObjNode.m_right);
          }
      }
      return false;
  }

  // check if two object in data_list are collided
  // assume there are exactly two objects
  IGL_INLINE void Viewer::check_collision() {
      if(treeNodesCollide(data_list[0].kd_tree, data_list[1].kd_tree))
          isActive = false;
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

    data().init(); // initiate object fields

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

  IGL_INLINE void Viewer::toggle_move()
  {
      isActive = !isActive;
  }

  IGL_INLINE void Viewer::init_scene()
  {

      int sign = 1;

      MyTranslate(Eigen::Vector3d(0, 0, -2), true);


      for (auto& obj : data_list) {

          obj.MyTranslate(Eigen::Vector3d(sign, 0, 0), true);

          obj.kd_tree.init(obj.V, obj.F);
          igl::AABB<Eigen::MatrixXd, 3> tree_first = obj.kd_tree;
          Eigen::AlignedBox<double, 3> box_first = tree_first.m_box;
          obj.drawAlignedBox(box_first, Eigen::RowVector3d(0, 0, 1));

          sign *= -1;
      }

      data_list[0].direction = 263; // move left
      data_list[1].direction = 265; // move left

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
