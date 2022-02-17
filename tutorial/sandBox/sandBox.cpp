#include "tutorial/sandBox/sandBox.h"
#include <iostream>

SandBox::SandBox()
{ }

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

	// load_mesh_from_file("C:/Users/tal74/projects/animation/animation3D/tutorial/data/cube.obj");
	// data().init();
	// data().drawAlignedBox(data().kd_tree.m_box, Eigen::RowVector3d(0, 1, 0));

    double z = -0.8 * scale;
    for (int i = 0; i <= joints_num; ++i)
    {
        skeleton.push_back(z * Eigen::Vector3d::UnitZ());
        z += 0.05 * scale;
    }

    calc_all_weights();
    data().MyRotate(Eigen::Vector3d::UnitY(), M_PI / 2);

    for (int i = 0; i <= joints_num; ++i)
    {
        split_snake.emplace_back();
        split_snake[i].MyTranslate(skeleton[i], true);
    }

    target_pose = skeleton[joints_num];
    U = V;

    MyTranslateInSystem(GetRotation(), Eigen::RowVector3d(0, 0, -10));

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
        move_targets();
        clean_data_list();
        check_level_up();

    }
}



