#include "tutorial/sandBox/sandBox.h"
#include <igl/dqs.h>
#include <iostream>

SandBox::SandBox()
{
    joints_num = 16;
    scale = 2;
}

void SandBox::Init(const std::string& config)
{
    std::string snake_data;
    std::ifstream snakeFile;

    vT.resize(17);
    vQ.resize(17);

    snakeFile.open(config);
    if (!snakeFile.is_open())
        std::cout << "Can't open file " << config << std::endl;
    else
    {
        while (snakeFile >> snake_data)
        {
            load_mesh_from_file(snake_data);
            parents.push_back(-1);
            data().show_overlay_depth = false;
            data().point_size = 10;
            data().line_width = 2;
            data().set_visible(false, 1);
            if (selected_data_index == 0)
                V = data().V;
        }
        snakeFile.close();
    }
    MyTranslate(-Eigen::Vector3d::UnitZ(), true);

    double z = -0.8 * scale;
    for (int i = 0; i <= joints_num; ++i)
    {
        skinnedSkeleton.push_back(z * Eigen::Vector3d::UnitZ());
        z += 0.1 * scale;
    }

    calc_all_weights();
    data().MyRotate(Eigen::Vector3d::UnitY(), M_PI / 2);

    target_pose = skinnedSkeleton[joints_num];
    U = V;
}

SandBox::~SandBox()
{

}

Eigen::VectorXd SandBox::create_weight_vec(double w1, double idx_w1, double w2, double idx_w2)
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

void SandBox::calc_all_weights()
{
    Eigen::MatrixXd V = data_list[0].V;
    int vertexNum = V.rows();
    W.resize(vertexNum, 17);

    double z_axe_coord, w_1, w_2, lower_bound, upper_bound;
    for (int i = 0; i < vertexNum; ++i) {
        z_axe_coord = V(i, 2);
        lower_bound = (floor(z_axe_coord * 10)) / 10;
        upper_bound = (ceil(z_axe_coord * 10)) / 10;
        w_1 = abs(z_axe_coord - upper_bound) * 10;
        w_2 = 1 - w_1;
        W.row(i) = create_weight_vec(w_1, lower_bound * 10 + 8, w_2, upper_bound * 10 + 8);
    }
}

void SandBox::next_vertices_position()
{
    vT[0] = skinnedSkeleton[0];
    for (int i = 0; i < joints_num; ++i) {
        vT[i+1] = skinnedSkeleton[i + 1];
        vT[i] += (vT[i + 1] - vT[i]) / 6;
    }
    vT[joints_num] += target_pose;
}

void SandBox::add_weights() {
    double distance;
    Eigen::MatrixXd V = data_list[0].V;
    int vertexNum = V.rows();
    W.resize(vertexNum, 17);

    for (int i = 0; i < vertexNum; ++i) {
        double related_distance = calc_related_distance(i);
        for (int j = 0; j < skinnedSkeleton.size(); ++j) {
            distance = abs(skinnedSkeleton[j].z() - V(i, 2));
            W(i, j) = pow((1 / distance), 4) / related_distance;
        }
        W.row(i).normalized();
    }
}

double SandBox::calc_related_distance(int i) {
    double sum = 0, distance;

    for (int j = 0; j < skinnedSkeleton.size(); ++j) {
        distance = abs(skinnedSkeleton[j].z() - data_list[0].V(i, 2));
        if (distance <= 0.1) 
            sum += pow((1 / distance), 4);
    }
    return sum;
}

void SandBox::move_snake(){
   double snake_speed = 0.05;

    if (direction != ' ')
    {
        switch (direction) {
        case 'l':
            target_pose = Eigen::Vector3d(0, 0, -snake_speed);
            break;
        case 'r':
            target_pose = Eigen::Vector3d(0, 0, snake_speed);
            break;
        case 'u':
            target_pose = Eigen::Vector3d(0, snake_speed, 0);
            break;
        case 'd':
            target_pose = Eigen::Vector3d(0, -snake_speed, 0);
            break;
        case 'w':
            target_pose = Eigen::Vector3d(snake_speed, 0, 0);
            break;
        case 's':
            target_pose = Eigen::Vector3d(-snake_speed, 0, 0);
            break;
        default:
            break;
        }

         next_vertices_position();
         igl::dqs(V, W, vQ, vT, U);
         data_list[0].set_vertices(U);

         for (int i = 0; i < skinnedSkeleton.size(); ++i)
             skinnedSkeleton[i] = vT[i];
    }
}

void SandBox::Animate()
{
         move_snake();
         generate_target();
         move_targets();
}


