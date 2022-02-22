#include "tutorial/sandBox/sandBox.h"
#include <iostream>

SandBox::SandBox()
{
    isTranslated = false;
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
            data().set_visible(false, 1);
            if (selected_data_index == 0)
                V = data().V;
            data().set_colors(Eigen::RowVector3d(1, 0.55, 0));
            data().image_texture("C:/Users/tal74/projects/animation/animation3D/tutorial/textures/snake.jpg");
        }
        snakeFile.close();
    }

    MyTranslate(-Eigen::Vector3d::UnitZ(), true);

    double z = -0.8 * scale;
    for (int i = 0; i <= joints_num; ++i)
    {
        skeleton.push_back(z * Eigen::Vector3d::UnitZ());
        z += 0.1 * scale;
    }

    weights_calc();

    data().MyRotate(Eigen::Vector3d::UnitY(), M_PI / 2);

    for (int i = 0; i < joints_num; ++i)
    {
        split_snake.emplace_back();
        Eigen::Vector3d currect_snake_skeleton = Eigen::Vector3d(skeleton.at(i)(2), skeleton.at(i)(1), skeleton.at(i)(0)); //snake_skeleton.at(i);// Eigen::Vector3d(snake_skeleton.at(i)(2), snake_skeleton.at(i)(1), snake_skeleton.at(i)(0));
        split_snake.at(i).MyTranslate(currect_snake_skeleton, true);
    }

    createJointBoxes();
	start_level();
    isActive = true;

}

SandBox::~SandBox()
{

}


void SandBox::Animate()
{


    if (isActive) {

        move_snake();
        generate_target();
        move_targets();
        clean_data_list();
        check_level_up();

    }
}



