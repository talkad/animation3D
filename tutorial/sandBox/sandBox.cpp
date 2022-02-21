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
            data().show_overlay_depth = false;
            data().point_size = 10;
            data().line_width = 2;
            data().set_visible(false, 1);
            if (selected_data_index == 0)
                V = data().V;
            data().image_texture("C:/Users/pijon/OneDrive/Desktop/animation3D/tutorial/textures/snake.jpg");
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

    calc_all_weights();

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

        //if (frames < 50)
        //    ++frames;
        //else {
        //    data_list[0].kd_tree.init(data_list[0].V, data_list[0].F);
        //    check_collision();
        //    frames = 0;
        //}

        move_snake();
        generate_target();
        //move_targets();
        clean_data_list();
        check_level_up();

    }
}



