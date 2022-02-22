// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_OPENGL_GLFW_VIEWER_H
#define IGL_OPENGL_GLFW_VIEWER_H

#ifndef IGL_OPENGL_4
#define IGL_OPENGL_4
#endif

#include "../../igl_inline.h"
#include "../MeshGL.h"

#include "../ViewerData.h"
#include "ViewerPlugin.h"


#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <string>
#include <cstdint>

#define IGL_MOD_SHIFT           0x0001
#define IGL_MOD_CONTROL         0x0002
#define IGL_MOD_ALT             0x0004
#define IGL_MOD_SUPER           0x0008



namespace igl
{
    namespace opengl
    {
        namespace glfw
        {
            // GLFW-based mesh viewer
            class Viewer : public Movable
            {
            public:

                // UI Enumerations
               // enum class MouseButton {Left, Middle, Right};
               // enum class MouseMode { None, Rotation, Zoom, Pan, Translation} mouse_mode;
                virtual void Init(const std::string config);
                virtual void Animate() {}
                virtual void WhenTranslate() {}
                virtual Eigen::Vector3d GetCameraPosition() { return Eigen::Vector3d(0, 0, 0); }
                virtual Eigen::Vector3d GetCameraForward() { return Eigen::Vector3d(0, 0, -1); }
                virtual Eigen::Vector3d GetCameraUp() { return Eigen::Vector3d(0, 1, 0); }

                //IGL_INLINE void init_plugins();
                //IGL_INLINE void shutdown_plugins();
                Viewer();
                virtual ~Viewer();
                // Mesh IO
                IGL_INLINE bool load_mesh_from_file(const std::string& mesh_file_name);
                IGL_INLINE bool save_mesh_to_file(const std::string& mesh_file_name);

                // Scene IO
                IGL_INLINE bool load_scene();
                IGL_INLINE bool load_scene(std::string fname);
                IGL_INLINE bool save_scene();
                IGL_INLINE bool save_scene(std::string fname);
                // Draw everything
               // IGL_INLINE void draw();
                // OpenGL context resize

                // Helper functions

                IGL_INLINE void open_dialog_load_mesh();
                IGL_INLINE void open_dialog_save_mesh();

                IGL_INLINE void draw() {}
                ////////////////////////
                // Multi-mesh methods //
                ////////////////////////

                // Return the current mesh, or the mesh corresponding to a given unique identifier
                //
                // Inputs:
                //   mesh_id  unique identifier associated to the desired mesh (current mesh if -1)
                IGL_INLINE ViewerData& data(int mesh_id = -1);
                IGL_INLINE const ViewerData& data(int mesh_id = -1) const;

                // Append a new "slot" for a mesh (i.e., create empty entries at the end of
                // the data_list and opengl_state_list.
                //
                // Inputs:
                //   visible  If true, the new mesh is set to be visible on all existing viewports
                // Returns the id of the last appended mesh
                //
                // Side Effects:
                //   selected_data_index is set this newly created, last entry (i.e.,
                //   #meshes-1)
                IGL_INLINE int append_mesh(bool visible = true);

                // Erase a mesh (i.e., its corresponding data and state entires in data_list
                // and opengl_state_list)
                //
                // Inputs:
                //   index  index of mesh to erase
                // Returns whether erasure was successful <=> cannot erase last mesh
                //
                // Side Effects:
                //   If selected_data_index is greater than or equal to index then it is
                //   decremented
                // Example:
                //   // Erase all mesh slots except first and clear remaining mesh
                //   viewer.selected_data_index = viewer.data_list.size()-1;
                //   while(viewer.erase_mesh(viewer.selected_data_index)){};
                //   viewer.data().clear();
                //
                IGL_INLINE bool erase_mesh(const size_t index);

                // Retrieve mesh index from its unique identifier
                // Returns 0 if not found
                IGL_INLINE size_t mesh_index(const int id) const;

                IGL_INLINE Eigen::Matrix4d CalcParentsTrans(int indx);

                IGL_INLINE Eigen::Matrix4d CalcSnakeJointsTrans();

                IGL_INLINE Eigen::Vector3d calcJointPos(int);

                void animateFABRIK();

                void fixAxis();

                IGL_INLINE Eigen::Matrix4d CalcParentsTranslation(int index);

                IGL_INLINE Eigen::Matrix3d CalcParentsInverseRotation(int index);

                IGL_INLINE void clean_data_list();

                bool boxes_collide(Eigen::AlignedBox<double, 3>& box1, Eigen::AlignedBox<double, 3>& box2, int i, int j);

                IGL_INLINE void move_targets();

                IGL_INLINE void generate_target();

                IGL_INLINE bool treeNodesCollide(AABB<Eigen::MatrixXd, 3>&, Eigen::AlignedBox<double, 3>&, int, int);

                // check if two object in data_list are collided

                IGL_INLINE void check_collision();

                IGL_INLINE void start_level();

                IGL_INLINE void check_level_up();

                IGL_INLINE void add_score(int);

                IGL_INLINE void update_timer();

                IGL_INLINE void Viewer::createJointBoxes();

                IGL_INLINE void move_snake();

                IGL_INLINE void weights_calc();

                IGL_INLINE Eigen::VectorXd create_weight_vec(double, double, double, double);

                IGL_INLINE void next_vertices_position();

                IGL_INLINE void add_weights();

                IGL_INLINE double relative_to_segments_distance(int i);


                inline bool SetAnimation() {
                    if (isActive)
                        fixAxis();
                    return isActive = !isActive;
                }
            public:
                //////////////////////
                // Member variables //
                //////////////////////

                // Alec: I call this data_list instead of just data to avoid confusion with
                // old "data" variable.
                // Stores all the data that should be visualized
                std::vector<ViewerData> data_list;

                std::vector<int> parents;

                size_t selected_data_index;
                int next_data_id;
                bool isPicked;
                bool isActive;
                Eigen::Vector4d tip;
                Eigen::Vector3d destination;

                int link_num;
                bool ikAnimation;
                double link_length;
                double snake_length;
                double snake_tail_start;
                double delta;
                int current_picked;

                int snake_size;
                bool snake_view;
                float prev_tic;


                Eigen::Vector3d position_offset;
                bool isGameOver;
                bool isLevelUp;
                bool start;
                bool isResume;
                bool isGameStarted;
                bool isPaused;

                int pause_time;
                int resume_time;
                int paused_time;

                int TTL;
                unsigned int frames = 0;
                int level;
                int score;
                int timer;
                int start_time;
                double p; // probability to generate target of type 1
                int target2_creation;
                int level1_obj_amount;
                int scale;
                int joints_num;
                std::vector<Eigen::Vector3d> skeleton;
                std::vector<Movable> split_snake;
                unsigned char direction;
                unsigned char previous_direction;

                bool isFog;
                bool isFP;

                bool update_camera_rotation;
                char keyPressed;
                int creation_gap;

                typedef
                    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
                    RotationList;
                std::vector<Eigen::AlignedBox<double, 3>> jointBoxes;
                // W - weights matrix
                // BE - Edges between joints
                // C - joints positions
                // P - parents
                // M - weights per vertex per joint matrix
                // U - new vertices position after skinning
                Eigen::MatrixXd V, W, C, U, M;
                Eigen::MatrixXi F, BE;
                Eigen::VectorXi P;
                RotationList vQ;
                std::vector<Eigen::Vector3d> vT;

                // Keep track of the global position of the scrollwheel
                float scroll_position;

            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };

        } // end namespace
    } // end namespace
} // end namespace

#ifndef IGL_STATIC_LIBRARY
#  include "Viewer.cpp"
#endif

#endif