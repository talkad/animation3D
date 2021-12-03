// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "ViewerData.h"
#include "ViewerCore.h"

#include "../per_face_normals.h"
#include "../material_colors.h"
#include "../parula.h"
#include "../per_vertex_normals.h"
#include "igl/png/texture_from_png.h"
#include <iostream>
//#include "external/stb/igl_stb_image.h"

#include <igl/edge_flaps.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/collapse_edge.h>
#include <igl/circulation.h>
#include <igl\vertex_triangle_adjacency.h>


IGL_INLINE bool igl::opengl::ViewerData::init_mesh()
{
    F = F_clone;
    V = V_clone;
    Q = new PriorityQueue();
    Q_iterator = new std::vector<PriorityQueue::iterator>();
    edge_col_num = 0;
    edge_flaps(F, E, EMAP, EF, EI); // fill in mappings between vertex and faces

    Q_iterator->resize(E.rows());

    C.resize(E.rows(), V.cols());
    Q->clear();
    
    Q_vertex_error.resize(V.rows());
    Quadratic_error_vertex(); // initiate vertices quadratic errors

    double cost = 0;
    Eigen::Vector3d p = Eigen::Vector3d::Zero(3);;

    for (int e_id = 0; e_id < E.rows(); e_id++)
    {
        new_cost_and_placement(e_id, V, F, E, EMAP, EF, EI, cost, p);

        C.row(e_id) = p;
        (*Q_iterator)[e_id] = Q->insert(std::pair<double, int>(cost, e_id)).first;
    }

    set_mesh(V, F);

    return true;
}


IGL_INLINE void igl::opengl::ViewerData::Simplification(int num_of_faces) {


    bool is_collapsed = false;
    int currenct_col_num = 0;

    for (int j = 0; j < num_of_faces; j++)
    {
        if (!new_collapse_edge(V, F, E, EMAP, EF, EI, *Q, *Q_iterator, C))
        //if (!new_collapse_edge(V, F))
        {
            break;
        }

        is_collapsed = true;
        edge_col_num++;
        currenct_col_num++;
    }

    if (is_collapsed)
    {
        Eigen::MatrixXd new_V = V;
        Eigen::MatrixXi new_F = F;

        clear();
        set_mesh(new_V, new_F);
        set_face_based(true);
        dirty = 157; //this line prevents texture coordinates update
    }
    
    std::cout << "num of total collapsed edges: " << edge_col_num << " | num of currenct collapsed edges: " << currenct_col_num << std::endl;
}

// calc Kp according to the article
IGL_INLINE Eigen::Matrix4d igl::opengl::ViewerData::calc_Kp(int vertex_index, int face_index) {
    Eigen::Vector3d norm = F_normals.row(face_index).normalized();
    double d = V.row(vertex_index) * norm;
    double a = norm[0], b = norm[1], c = norm[2];
    d *= -1;

    Eigen::Matrix4d Kp;
    Kp.row(0) = Eigen::Vector4d(a * a, a * b, a * c, a * d);
    Kp.row(1) = Eigen::Vector4d(a * b, b * b, b * c, b * d);
    Kp.row(2) = Eigen::Vector4d(a * c, b * c, c * c, c * d);
    Kp.row(3) = Eigen::Vector4d(a * d, d * b, d * c, d * d);

    return Kp;
}

IGL_INLINE void igl::opengl::ViewerData::Quadratic_error_vertex() {

    Eigen::MatrixXd V = V_clone;
    Eigen::MatrixXi F = F_clone;
    std::vector<std::vector<int> > VF;
    std::vector<std::vector<int> > VFi;

    igl::vertex_triangle_adjacency(V, F, VF, VFi);    // constructs the vertex-face topology of a given mesh

    for (int vi = 0; vi < V.rows(); vi++) {

        Q_vertex_error[vi] = Eigen::Matrix4d::Zero(); // initializing quadratic error for each vertex with zero

        for (int fi = 0; fi < VF[vi].size(); fi++) {

            Q_vertex_error[vi] += calc_Kp(vi, VF[vi][fi]);
        }

    }
}

IGL_INLINE void igl::opengl::ViewerData::new_cost_and_placement(
    int e,
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& /*F*/,
    Eigen::MatrixXi& E,
    Eigen::VectorXi& /*EMAP*/,
    Eigen::MatrixXi& /*EF*/,
    Eigen::MatrixXi& /*EI*/,
    double& cost,
    Eigen::Vector3d& p)
{
    int vertex1_id = E(e, 0);
    int vertex2_id = E(e, 1);

    Eigen::Matrix4d Q = Q_vertex_error[vertex1_id] + Q_vertex_error[vertex2_id]; 

    Eigen::Matrix4d Q_grad = Q;
    Q_grad.row(3) = Eigen::Vector4d(0, 0, 0, 1);   // The bottom row of the matrix is empty because v¯ is an homogeneous vector

    bool is_invertable;
    Eigen::Vector4d::Scalar det;
    double threshold;
    Q_grad.computeInverseAndDetWithCheck(Q_grad, det, is_invertable, threshold);

    if (is_invertable) {
        Eigen::Vector4d v_hat;

        v_hat = Q_grad * (Eigen::Vector4d(0, 0, 0, 1));
        p[0] = v_hat[0], p[1] = v_hat[1], p[2] = v_hat[2];
        cost = v_hat.transpose() * Q * v_hat;

    }
    // find the optimal vertex along the segment between the vertices
    else { 
        int num_of_segment = 9;

        Eigen::Vector3d v1 = V.row(vertex1_id);
        Eigen::Vector3d v2 = V.row(vertex2_id);

        Eigen::Vector3d base_vertex = v1;
        Eigen::Vector4d current_p = Eigen::Vector4d(base_vertex[0], base_vertex[1], base_vertex[2], 1);
        double currenct_cost = current_p.transpose() * Q * current_p; 

        Eigen::Vector4d opt_p = current_p;  
        double min_cost = currenct_cost;

        double d_x = (v2[0] - v1[0]) / num_of_segment;
        double d_y = (v2[1] - v1[1]) / num_of_segment;
        double d_z = (v2[2] - v1[2]) / num_of_segment;

        for (int i = 1; i <= num_of_segment + 1; i++) {
            current_p = Eigen::Vector4d(base_vertex[0] + d_x * i, base_vertex[1] + d_y * i, base_vertex[2] + d_z * i, 1);
            currenct_cost = current_p.transpose() * Q * current_p;

            if (currenct_cost < min_cost) {
                min_cost = currenct_cost;
                opt_p = current_p;
            }
        }

        p[0] = opt_p[0], p[1] = opt_p[1], p[2] = opt_p[2];
        cost = min_cost;
    }

    ///* naive approach
    //
    //else {
    //    p = 0.5 * (V.row(vertex1_id) + V.row(vertex2_id));
    //    // v_hat[0] = p[0], v_hat[1] = p[1], v_hat[2] = p[2], v_hat[3] = 1;

    //    cost = (V.row(vertex1_id) - V.row(vertex2_id)).norm();
    //}
    //*/
}

IGL_INLINE bool igl::opengl::ViewerData::new_collapse_edge(
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& F,
    Eigen::MatrixXi& E,
    Eigen::VectorXi& EMAP,
    Eigen::MatrixXi& EF,
    Eigen::MatrixXi& EI,
    std::set<std::pair<double, int> >& Q,
    std::vector<std::set<std::pair<double, int> >::iterator >& Qit,
    Eigen::MatrixXd& C)
{
    std::pair<double, int> edge;
    if (Q.empty() || (edge = *(Q.begin())).first == std::numeric_limits<double>::infinity()) // Guard - check if there is an edge to collapse
        return false;                                                                        // If exists and has finite cost - a. Takes out the loewst cost edge from queue. 

    Q.erase(Q.begin()); // b. Deletes edge
    Qit[edge.second] = Q.end();

    int vertex1_idx = E.row(edge.second)[0],
        vertex2_idx = E.row(edge.second)[1],
        e1, e2, f1, f2;
    double cost = 0;
    Eigen::Vector3d new_p = Eigen::Vector3d::Zero(3);
    std::vector<int> nf = circulation(edge.second, true, EMAP, EF, EI);
    std::vector<int> opposite_nf = circulation(edge.second, false, EMAP, EF, EI);

    nf.insert(nf.begin(), opposite_nf.begin(), opposite_nf.end());

    // Steps c and d are performed in igl function collapse_edge
    if (igl::collapse_edge(edge.second, C.row(edge.second), V, F, E, EMAP, EF, EI, e1, e2, f1, f2)) {
        Q_vertex_error[vertex1_idx] = Q_vertex_error[vertex2_idx] =
            Q_vertex_error[vertex1_idx] + Q_vertex_error[vertex2_idx]; // e. Updates new vertex edges errors

        Q.erase(Qit[e1]);
        Q.erase(Qit[e2]);
        Qit[e1] = Q.end();
        Qit[e2] = Q.end();

        for (auto face : nf) {
            if (F(face, 0) != IGL_COLLAPSE_EDGE_NULL ||
                F(face, 1) != IGL_COLLAPSE_EDGE_NULL ||
                F(face, 2) != IGL_COLLAPSE_EDGE_NULL)
            {
                for (int v = 0; v < 3; ++v) {
                    int ei = EMAP(v * F.rows() + face);
                    Q.erase(Qit[ei]);
                    new_cost_and_placement(ei, V, F, E, EMAP, EF, EI, cost, new_p);
                    Qit[ei] = Q.insert(std::pair<double, int>(cost, ei)).first;
                    C.row(ei) = new_p;
                }
            }
        }

        std::cout << "edge " << edge.second << " cost = " << cost << ", new v position (" << new_p[0] << ","
            << new_p[1] << "," << new_p[2] << ")" << std::endl;

    }
    else {
        edge.first = std::numeric_limits<double>::infinity();
        Qit[edge.second] = Q.insert(edge).first;
    }
    return true;
}

IGL_INLINE igl::opengl::ViewerData::ViewerData()
: dirty(MeshGL::DIRTY_ALL),
  show_faces(true),
  show_lines(true),
  invert_normals(false),
  show_overlay(true),
  show_overlay_depth(true),
  show_vertid(false),
  show_faceid(false),
  show_texture(false),
  point_size(30),
  line_width(0.5f),
  line_color(0,0,0,1),
  label_color(0,0,0.04,1),
  shininess(35.0f),
  id(-1),
  is_visible(1)
{
  clear();
};

IGL_INLINE void igl::opengl::ViewerData::set_face_based(bool newvalue)
{
  if (face_based != newvalue)
  {
    face_based = newvalue;
    dirty = MeshGL::DIRTY_ALL;
  }
}

// Helpers that draws the most common meshes
IGL_INLINE void igl::opengl::ViewerData::set_mesh(
    const Eigen::MatrixXd& _V, const Eigen::MatrixXi& _F)
{
  using namespace std;

  Eigen::MatrixXd V_temp;

  // If V only has two columns, pad with a column of zeros
  if (_V.cols() == 2)
  {
    V_temp = Eigen::MatrixXd::Zero(_V.rows(),3);
    V_temp.block(0,0,_V.rows(),2) = _V;
  }
  else
    V_temp = _V;

  if (V.rows() == 0 && F.rows() == 0)
  {
    V = V_temp;
    F = _F;

    compute_normals();
    uniform_colors(
      Eigen::Vector3d(GOLD_AMBIENT[0], GOLD_AMBIENT[1], GOLD_AMBIENT[2]),
      Eigen::Vector3d(GOLD_DIFFUSE[0], GOLD_DIFFUSE[1], GOLD_DIFFUSE[2]),
      Eigen::Vector3d(GOLD_SPECULAR[0], GOLD_SPECULAR[1], GOLD_SPECULAR[2]));
	image_texture("C:/Users/tal74/animation/EngineForAnimationCourse/tutorial/textures/snake1.png");
//    grid_texture();
  }
  else
  {
    if (_V.rows() == V.rows() && _F.rows() == F.rows())
    {
      V = V_temp;
      F = _F;
    }
    else
      cerr << "ERROR (set_mesh): The new mesh has a different number of vertices/faces. Please clear the mesh before plotting."<<endl;
  }
  dirty |= MeshGL::DIRTY_FACE | MeshGL::DIRTY_POSITION;
}

IGL_INLINE void igl::opengl::ViewerData::set_vertices(const Eigen::MatrixXd& _V)
{
  V = _V;
  assert(F.size() == 0 || F.maxCoeff() < V.rows());
  dirty |= MeshGL::DIRTY_POSITION;
}

IGL_INLINE void igl::opengl::ViewerData::set_normals(const Eigen::MatrixXd& N)
{
  using namespace std;
  if (N.rows() == V.rows())
  {
    set_face_based(false);
    V_normals = N;
  }
  else if (N.rows() == F.rows() || N.rows() == F.rows()*3)
  {
    set_face_based(true);
    F_normals = N;
  }
  else
    cerr << "ERROR (set_normals): Please provide a normal per face, per corner or per vertex."<<endl;
  dirty |= MeshGL::DIRTY_NORMAL;
}

IGL_INLINE void igl::opengl::ViewerData::set_visible(bool value, unsigned int core_id /*= 1*/)
{
  if (value)
    is_visible |= core_id;
  else
  is_visible &= ~core_id;
}

//IGL_INLINE void igl::opengl::ViewerData::copy_options(const ViewerCore &from, const ViewerCore &to)
//{
//  to.set(show_overlay      , from.is_set(show_overlay)      );
//  to.set(show_overlay_depth, from.is_set(show_overlay_depth));
//  to.set(show_texture      , from.is_set(show_texture)      );
//  to.set(show_faces        , from.is_set(show_faces)        );
//  to.set(show_lines        , from.is_set(show_lines)        );
//}

IGL_INLINE void igl::opengl::ViewerData::set_colors(const Eigen::MatrixXd &C)
{
  using namespace std;
  using namespace Eigen;
  if(C.rows()>0 && C.cols() == 1)
  {
    Eigen::MatrixXd C3;
    igl::parula(C,true,C3);
    return set_colors(C3);
  }
  // Ambient color should be darker color
  const auto ambient = [](const MatrixXd & C)->MatrixXd
  {
    MatrixXd T = 0.1*C;
    T.col(3) = C.col(3);
    return T;
  };
  // Specular color should be a less saturated and darker color: dampened
  // highlights
  const auto specular = [](const MatrixXd & C)->MatrixXd
  {
    const double grey = 0.3;
    MatrixXd T = grey+0.1*(C.array()-grey);
    T.col(3) = C.col(3);
    return T;
  };
  if (C.rows() == 1)
  {
    for (unsigned i=0;i<V_material_diffuse.rows();++i)
    {
      if (C.cols() == 3)
        V_material_diffuse.row(i) << C.row(0),1;
      else if (C.cols() == 4)
        V_material_diffuse.row(i) << C.row(0);
    }
    V_material_ambient = ambient(V_material_diffuse);
    V_material_specular = specular(V_material_diffuse);

    for (unsigned i=0;i<F_material_diffuse.rows();++i)
    {
      if (C.cols() == 3)
        F_material_diffuse.row(i) << C.row(0),1;
      else if (C.cols() == 4)
        F_material_diffuse.row(i) << C.row(0);
    }
    F_material_ambient = ambient(F_material_diffuse);
    F_material_specular = specular(F_material_diffuse);
  }
  else if (C.rows() == V.rows())
  {
    set_face_based(false);
    for (unsigned i=0;i<V_material_diffuse.rows();++i)
    {
      if (C.cols() == 3)
        V_material_diffuse.row(i) << C.row(i), 1;
      else if (C.cols() == 4)
        V_material_diffuse.row(i) << C.row(i);
    }
    V_material_ambient = ambient(V_material_diffuse);
    V_material_specular = specular(V_material_diffuse);
  }
  else if (C.rows() == F.rows())
  {
    set_face_based(true);
    for (unsigned i=0;i<F_material_diffuse.rows();++i)
    {
      if (C.cols() == 3)
        F_material_diffuse.row(i) << C.row(i), 1;
      else if (C.cols() == 4)
        F_material_diffuse.row(i) << C.row(i);
    }
    F_material_ambient = ambient(F_material_diffuse);
    F_material_specular = specular(F_material_diffuse);
  }
  else
    cerr << "ERROR (set_colors): Please provide a single color, or a color per face or per vertex."<<endl;
  dirty |= MeshGL::DIRTY_DIFFUSE;

}

IGL_INLINE void igl::opengl::ViewerData::set_uv(const Eigen::MatrixXd& UV)
{
  using namespace std;
  if (UV.rows() == V.rows())
  {
    set_face_based(false);
    V_uv = UV;
  }
  else
    cerr << "ERROR (set_UV): Please provide uv per vertex."<<endl;;
  dirty |= MeshGL::DIRTY_UV;
}

IGL_INLINE void igl::opengl::ViewerData::set_uv(const Eigen::MatrixXd& UV_V, const Eigen::MatrixXi& UV_F)
{
  set_face_based(true);
  V_uv = UV_V.block(0,0,UV_V.rows(),2);
  F_uv = UV_F;
  dirty |= MeshGL::DIRTY_UV;
}

IGL_INLINE void igl::opengl::ViewerData::set_texture(
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& R,
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& G,
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& B)
{
  texture_R = R;
  texture_G = G;
  texture_B = B;
  texture_A = Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>::Constant(R.rows(),R.cols(),255);
  dirty |= MeshGL::DIRTY_TEXTURE;
}

IGL_INLINE void igl::opengl::ViewerData::set_texture(
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& R,
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& G,
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& B,
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& A)
{
  texture_R = R;
  texture_G = G;
  texture_B = B;
  texture_A = A;
  dirty |= MeshGL::DIRTY_TEXTURE;
}

IGL_INLINE void igl::opengl::ViewerData::set_points(
  const Eigen::MatrixXd& P,
  const Eigen::MatrixXd& C)
{
  // clear existing points
  points.resize(0,0);
  add_points(P,C);
}

IGL_INLINE void igl::opengl::ViewerData::add_points(const Eigen::MatrixXd& P,  const Eigen::MatrixXd& C)
{
  Eigen::MatrixXd P_temp;

  // If P only has two columns, pad with a column of zeros
  if (P.cols() == 2)
  {
    P_temp = Eigen::MatrixXd::Zero(P.rows(),3);
    P_temp.block(0,0,P.rows(),2) = P;
  }
  else
    P_temp = P;

  int lastid = points.rows();
  points.conservativeResize(points.rows() + P_temp.rows(),6);
  for (unsigned i=0; i<P_temp.rows(); ++i)
    points.row(lastid+i) << P_temp.row(i), i<C.rows() ? C.row(i) : C.row(C.rows()-1);

  dirty |= MeshGL::DIRTY_OVERLAY_POINTS;
}

IGL_INLINE void igl::opengl::ViewerData::set_edges(
  const Eigen::MatrixXd& P,
  const Eigen::MatrixXi& E,
  const Eigen::MatrixXd& C)
{
  using namespace Eigen;
  lines.resize(E.rows(),9);
  assert(C.cols() == 3);
  for(int e = 0;e<E.rows();e++)
  {
    RowVector3d color;
    if(C.size() == 3)
    {
      color<<C;
    }else if(C.rows() == E.rows())
    {
      color<<C.row(e);
    }
    lines.row(e)<< P.row(E(e,0)), P.row(E(e,1)), color;
  }
  dirty |= MeshGL::DIRTY_OVERLAY_LINES;
}

IGL_INLINE void igl::opengl::ViewerData::add_edges(const Eigen::MatrixXd& P1, const Eigen::MatrixXd& P2, const Eigen::MatrixXd& C)
{
  Eigen::MatrixXd P1_temp,P2_temp;

  // If P1 only has two columns, pad with a column of zeros
  if (P1.cols() == 2)
  {
    P1_temp = Eigen::MatrixXd::Zero(P1.rows(),3);
    P1_temp.block(0,0,P1.rows(),2) = P1;
    P2_temp = Eigen::MatrixXd::Zero(P2.rows(),3);
    P2_temp.block(0,0,P2.rows(),2) = P2;
  }
  else
  {
    P1_temp = P1;
    P2_temp = P2;
  }

  int lastid = lines.rows();
  lines.conservativeResize(lines.rows() + P1_temp.rows(),9);
  for (unsigned i=0; i<P1_temp.rows(); ++i)
    lines.row(lastid+i) << P1_temp.row(i), P2_temp.row(i), i<C.rows() ? C.row(i) : C.row(C.rows()-1);

  dirty |= MeshGL::DIRTY_OVERLAY_LINES;
}

IGL_INLINE void igl::opengl::ViewerData::add_label(const Eigen::VectorXd& P,  const std::string& str)
{
  Eigen::RowVectorXd P_temp;

  // If P only has two columns, pad with a column of zeros
  if (P.size() == 2)
  {
    P_temp = Eigen::RowVectorXd::Zero(3);
    P_temp << P.transpose(), 0;
  }
  else
    P_temp = P;

  int lastid = labels_positions.rows();
  labels_positions.conservativeResize(lastid+1, 3);
  labels_positions.row(lastid) = P_temp;
  labels_strings.push_back(str);
}

IGL_INLINE void igl::opengl::ViewerData::clear_labels()
{
  labels_positions.resize(0,3);
  labels_strings.clear();
}

IGL_INLINE void igl::opengl::ViewerData::clear()
{
  V                       = Eigen::MatrixXd (0,3);
  F                       = Eigen::MatrixXi (0,3);

  F_material_ambient      = Eigen::MatrixXd (0,4);
  F_material_diffuse      = Eigen::MatrixXd (0,4);
  F_material_specular     = Eigen::MatrixXd (0,4);

  V_material_ambient      = Eigen::MatrixXd (0,4);
  V_material_diffuse      = Eigen::MatrixXd (0,4);
  V_material_specular     = Eigen::MatrixXd (0,4);

  F_normals               = Eigen::MatrixXd (0,3);
  V_normals               = Eigen::MatrixXd (0,3);

  V_uv                    = Eigen::MatrixXd (0,2);
  F_uv                    = Eigen::MatrixXi (0,3);

  lines                   = Eigen::MatrixXd (0,9);
  points                  = Eigen::MatrixXd (0,6);
  labels_positions        = Eigen::MatrixXd (0,3);
  labels_strings.clear();

  face_based = false;
}

IGL_INLINE void igl::opengl::ViewerData::compute_normals()
{
  igl::per_face_normals(V, F, F_normals);
  igl::per_vertex_normals(V, F, F_normals, V_normals);
  dirty |= MeshGL::DIRTY_NORMAL;
}

IGL_INLINE void igl::opengl::ViewerData::uniform_colors(
  const Eigen::Vector3d& ambient,
  const Eigen::Vector3d& diffuse,
  const Eigen::Vector3d& specular)
{
  Eigen::Vector4d ambient4;
  Eigen::Vector4d diffuse4;
  Eigen::Vector4d specular4;

  ambient4 << ambient, 1;
  diffuse4 << diffuse, 1;
  specular4 << specular, 1;

  uniform_colors(ambient4,diffuse4,specular4);
}

IGL_INLINE void igl::opengl::ViewerData::uniform_colors(
  const Eigen::Vector4d& ambient,
  const Eigen::Vector4d& diffuse,
  const Eigen::Vector4d& specular)
{
  V_material_ambient.resize(V.rows(),4);
  V_material_diffuse.resize(V.rows(),4);
  V_material_specular.resize(V.rows(),4);

  for (unsigned i=0; i<V.rows();++i)
  {
    V_material_ambient.row(i) = ambient;
    V_material_diffuse.row(i) = diffuse;
    V_material_specular.row(i) = specular;
  }

  F_material_ambient.resize(F.rows(),4);
  F_material_diffuse.resize(F.rows(),4);
  F_material_specular.resize(F.rows(),4);

  for (unsigned i=0; i<F.rows();++i)
  {
    F_material_ambient.row(i) = ambient;
    F_material_diffuse.row(i) = diffuse;
    F_material_specular.row(i) = specular;
  }
  dirty |= MeshGL::DIRTY_SPECULAR | MeshGL::DIRTY_DIFFUSE | MeshGL::DIRTY_AMBIENT;
}

IGL_INLINE void igl::opengl::ViewerData::image_texture(const std::string fileName)
{
	//unsigned int texId;
	//if (igl::png::texture_from_png(fileName, false, texId))
	if(igl::png::texture_from_png(fileName,texture_R, texture_G, texture_B, texture_A))
	
		dirty |= MeshGL::DIRTY_TEXTURE;
	else
		std::cout<<"can't open texture file"<<std::endl;
}

IGL_INLINE void igl::opengl::ViewerData::grid_texture()
{
  // Don't do anything for an empty mesh
  if(V.rows() == 0)
  {
    V_uv.resize(V.rows(),2);
    return;
  }
  if (V_uv.rows() == 0)
  {
    V_uv = V.block(0, 0, V.rows(), 2);
    V_uv.col(0) = V_uv.col(0).array() - V_uv.col(0).minCoeff();
    V_uv.col(0) = V_uv.col(0).array() / V_uv.col(0).maxCoeff();
    V_uv.col(1) = V_uv.col(1).array() - V_uv.col(1).minCoeff();
    V_uv.col(1) = V_uv.col(1).array() / V_uv.col(1).maxCoeff();
    V_uv = V_uv.array() * 10;
    dirty |= MeshGL::DIRTY_TEXTURE;
  }

  unsigned size = 4;
  unsigned size2 = size/2;
  texture_R.resize(size, size);
  for (unsigned i=0; i<size; ++i)
  {
    for (unsigned j=0; j<size; ++j)
    {
      texture_R(i,j) = 0;
      if ((i<size2 && j<size2) || (i>=size2 && j>=size2))
        texture_R(i,j) = 255;
    }
  }

  texture_G = texture_R;
  texture_B = texture_R;
  texture_A = Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>::Constant(texture_R.rows(),texture_R.cols(),255);
  dirty |= MeshGL::DIRTY_TEXTURE;
}

IGL_INLINE void igl::opengl::ViewerData::updateGL(
  const igl::opengl::ViewerData& data,
  const bool invert_normals,
  igl::opengl::MeshGL& meshgl
  )
{
  if (!meshgl.is_initialized)
  {
    meshgl.init();
  }

  bool per_corner_uv = (data.F_uv.rows() == data.F.rows());
  bool per_corner_normals = (data.F_normals.rows() == 3 * data.F.rows());

  meshgl.dirty |= data.dirty;

  // Input:
  //   X  #F by dim quantity
  // Output:
  //   X_vbo  #F*3 by dim scattering per corner
  const auto per_face = [&data](
      const Eigen::MatrixXd & X,
      MeshGL::RowMatrixXf & X_vbo)
  {
    assert(X.cols() == 4);
    X_vbo.resize(data.F.rows()*3,4);
    for (unsigned i=0; i<data.F.rows();++i)
      for (unsigned j=0;j<3;++j)
        X_vbo.row(i*3+j) = X.row(i).cast<float>();
  };

  // Input:
  //   X  #V by dim quantity
  // Output:
  //   X_vbo  #F*3 by dim scattering per corner
  const auto per_corner = [&data](
      const Eigen::MatrixXd & X,
      MeshGL::RowMatrixXf & X_vbo)
  {
    X_vbo.resize(data.F.rows()*3,X.cols());
    for (unsigned i=0; i<data.F.rows();++i)
      for (unsigned j=0;j<3;++j)
        X_vbo.row(i*3+j) = X.row(data.F(i,j)).cast<float>();
  };

  if (!data.face_based)
  {
    if (!(per_corner_uv || per_corner_normals))
    {
      // Vertex positions
      if (meshgl.dirty & MeshGL::DIRTY_POSITION)
        meshgl.V_vbo = data.V.cast<float>();

      // Vertex normals
      if (meshgl.dirty & MeshGL::DIRTY_NORMAL)
      {
        meshgl.V_normals_vbo = data.V_normals.cast<float>();
        if (invert_normals)
          meshgl.V_normals_vbo = -meshgl.V_normals_vbo;
      }

      // Per-vertex material settings
      if (meshgl.dirty & MeshGL::DIRTY_AMBIENT)
        meshgl.V_ambient_vbo = data.V_material_ambient.cast<float>();
      if (meshgl.dirty & MeshGL::DIRTY_DIFFUSE)
        meshgl.V_diffuse_vbo = data.V_material_diffuse.cast<float>();
      if (meshgl.dirty & MeshGL::DIRTY_SPECULAR)
        meshgl.V_specular_vbo = data.V_material_specular.cast<float>();

      // Face indices
      if (meshgl.dirty & MeshGL::DIRTY_FACE)
        meshgl.F_vbo = data.F.cast<unsigned>();

      // Texture coordinates
      if (meshgl.dirty & MeshGL::DIRTY_UV)
      {
        meshgl.V_uv_vbo = data.V_uv.cast<float>();
      }
    }
    else
    {

      // Per vertex properties with per corner UVs
      if (meshgl.dirty & MeshGL::DIRTY_POSITION)
      {
        per_corner(data.V,meshgl.V_vbo);
      }

      if (meshgl.dirty & MeshGL::DIRTY_AMBIENT)
      {
        meshgl.V_ambient_vbo.resize(data.F.rows()*3,4);
        for (unsigned i=0; i<data.F.rows();++i)
          for (unsigned j=0;j<3;++j)
            meshgl.V_ambient_vbo.row(i*3+j) = data.V_material_ambient.row(data.F(i,j)).cast<float>();
      }
      if (meshgl.dirty & MeshGL::DIRTY_DIFFUSE)
      {
        meshgl.V_diffuse_vbo.resize(data.F.rows()*3,4);
        for (unsigned i=0; i<data.F.rows();++i)
          for (unsigned j=0;j<3;++j)
            meshgl.V_diffuse_vbo.row(i*3+j) = data.V_material_diffuse.row(data.F(i,j)).cast<float>();
      }
      if (meshgl.dirty & MeshGL::DIRTY_SPECULAR)
      {
        meshgl.V_specular_vbo.resize(data.F.rows()*3,4);
        for (unsigned i=0; i<data.F.rows();++i)
          for (unsigned j=0;j<3;++j)
            meshgl.V_specular_vbo.row(i*3+j) = data.V_material_specular.row(data.F(i,j)).cast<float>();
      }

      if (meshgl.dirty & MeshGL::DIRTY_NORMAL)
      {
        meshgl.V_normals_vbo.resize(data.F.rows()*3,3);
        for (unsigned i=0; i<data.F.rows();++i)
          for (unsigned j=0;j<3;++j)
            meshgl.V_normals_vbo.row(i*3+j) =
                         per_corner_normals ?
               data.F_normals.row(i*3+j).cast<float>() :
               data.V_normals.row(data.F(i,j)).cast<float>();


        if (invert_normals)
          meshgl.V_normals_vbo = -meshgl.V_normals_vbo;
      }

      if (meshgl.dirty & MeshGL::DIRTY_FACE)
      {
        meshgl.F_vbo.resize(data.F.rows(),3);
        for (unsigned i=0; i<data.F.rows();++i)
          meshgl.F_vbo.row(i) << i*3+0, i*3+1, i*3+2;
      }

      if (meshgl.dirty & MeshGL::DIRTY_UV)
      {
        meshgl.V_uv_vbo.resize(data.F.rows()*3,2);
        for (unsigned i=0; i<data.F.rows();++i)
          for (unsigned j=0;j<3;++j)
            meshgl.V_uv_vbo.row(i*3+j) =
              data.V_uv.row(per_corner_uv ?
                data.F_uv(i,j) : data.F(i,j)).cast<float>();
      }
    }
  }
  else
  {
    if (meshgl.dirty & MeshGL::DIRTY_POSITION)
    {
      per_corner(data.V,meshgl.V_vbo);
    }
    if (meshgl.dirty & MeshGL::DIRTY_AMBIENT)
    {
      per_face(data.F_material_ambient,meshgl.V_ambient_vbo);
    }
    if (meshgl.dirty & MeshGL::DIRTY_DIFFUSE)
    {
      per_face(data.F_material_diffuse,meshgl.V_diffuse_vbo);
    }
    if (meshgl.dirty & MeshGL::DIRTY_SPECULAR)
    {
      per_face(data.F_material_specular,meshgl.V_specular_vbo);
    }

    if (meshgl.dirty & MeshGL::DIRTY_NORMAL)
    {
      meshgl.V_normals_vbo.resize(data.F.rows()*3,3);
      for (unsigned i=0; i<data.F.rows();++i)
        for (unsigned j=0;j<3;++j)
          meshgl.V_normals_vbo.row(i*3+j) =
             per_corner_normals ?
               data.F_normals.row(i*3+j).cast<float>() :
               data.F_normals.row(i).cast<float>();

      if (invert_normals)
        meshgl.V_normals_vbo = -meshgl.V_normals_vbo;
    }

    if (meshgl.dirty & MeshGL::DIRTY_FACE)
    {
      meshgl.F_vbo.resize(data.F.rows(),3);
      for (unsigned i=0; i<data.F.rows();++i)
        meshgl.F_vbo.row(i) << i*3+0, i*3+1, i*3+2;
    }

    if (meshgl.dirty & MeshGL::DIRTY_UV)
    {
        meshgl.V_uv_vbo.resize(data.F.rows()*3,2);
        for (unsigned i=0; i<data.F.rows();++i)
          for (unsigned j=0;j<3;++j)
            meshgl.V_uv_vbo.row(i*3+j) = data.V_uv.row(per_corner_uv ? data.F_uv(i,j) : data.F(i,j)).cast<float>();
    }
  }

  if (meshgl.dirty & MeshGL::DIRTY_TEXTURE)
  {
    meshgl.tex_u = data.texture_R.rows();
    meshgl.tex_v = data.texture_R.cols();
    meshgl.tex.resize(data.texture_R.size()*4);
    for (unsigned i=0;i<data.texture_R.size();++i)
    {
      meshgl.tex(i*4+0) = data.texture_R(i);
      meshgl.tex(i*4+1) = data.texture_G(i);
      meshgl.tex(i*4+2) = data.texture_B(i);
      meshgl.tex(i*4+3) = data.texture_A(i);
    }
  }

  if (meshgl.dirty & MeshGL::DIRTY_OVERLAY_LINES)
  {
    meshgl.lines_V_vbo.resize(data.lines.rows()*2,3);
    meshgl.lines_V_colors_vbo.resize(data.lines.rows()*2,3);
    meshgl.lines_F_vbo.resize(data.lines.rows()*2,1);
    for (unsigned i=0; i<data.lines.rows();++i)
    {
      meshgl.lines_V_vbo.row(2*i+0) = data.lines.block<1, 3>(i, 0).cast<float>();
      meshgl.lines_V_vbo.row(2*i+1) = data.lines.block<1, 3>(i, 3).cast<float>();
      meshgl.lines_V_colors_vbo.row(2*i+0) = data.lines.block<1, 3>(i, 6).cast<float>();
      meshgl.lines_V_colors_vbo.row(2*i+1) = data.lines.block<1, 3>(i, 6).cast<float>();
      meshgl.lines_F_vbo(2*i+0) = 2*i+0;
      meshgl.lines_F_vbo(2*i+1) = 2*i+1;
    }
  }

  if (meshgl.dirty & MeshGL::DIRTY_OVERLAY_POINTS)
  {
    meshgl.points_V_vbo.resize(data.points.rows(),3);
    meshgl.points_V_colors_vbo.resize(data.points.rows(),3);
    meshgl.points_F_vbo.resize(data.points.rows(),1);
    for (unsigned i=0; i<data.points.rows();++i)
    {
      meshgl.points_V_vbo.row(i) = data.points.block<1, 3>(i, 0).cast<float>();
      meshgl.points_V_colors_vbo.row(i) = data.points.block<1, 3>(i, 3).cast<float>();
      meshgl.points_F_vbo(i) = i;
    }
  }
}
