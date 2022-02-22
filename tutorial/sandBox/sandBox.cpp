#include "tutorial/sandBox/sandBox.h"
#include <iostream>

SandBox::SandBox(){}

void SandBox::Init(const std::string& config)
{
    std::string snake_data;
    std::ifstream snakeFile;

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
            data().image_texture("C:/Users/pijon/OneDrive/Desktop/animation3D/tutorial/textures/snake.jpg");
        }
        snakeFile.close();
    }

    MyTranslate(-Eigen::Vector3d::UnitZ(), true);

    double z = snake_tail_start, snake_link_len = link_length / joints_num;
    for (int i = 0; i <= joints_num; ++i)
    {
        skeleton.push_back(z * Eigen::Vector3d::UnitZ());
        z += snake_link_len;
    }

    weights_calc();

    data().MyRotate(Eigen::Vector3d::UnitY(), M_PI / 2);

    for (int i = 0; i < joints_num; ++i)
    {
        split_snake.emplace_back();
        split_snake[i].MyTranslate(skeleton[i].reverse(), true);
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



