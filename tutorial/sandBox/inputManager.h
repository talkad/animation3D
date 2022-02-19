#pragma once
#include "igl/opengl/glfw/Display.h"
#include "igl/opengl/glfw/Renderer.h"
#include "sandBox.h"
#include "igl/look_at.h"
//#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
//#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
//#include <../imgui/imgui.h>



void glfw_window_size(GLFWwindow* window, int width, int height)
{
	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	//igl::opengl::glfw::Viewer* scn = rndr->GetScene();

    rndr->post_resize(window,width, height);

}

//static void glfw_drop_callback(GLFWwindow *window,int count,const char **filenames)
//{
//
//}

//static void glfw_error_callback(int error, const char* description)
//{
//	fputs(description, stderr);
//}

static void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int modifier)
{
	Renderer* rndr = (Renderer*) glfwGetWindowUserPointer(window);
	SandBox* scn = (SandBox*)rndr->GetScene();
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);

	else if(action == GLFW_PRESS || action == GLFW_REPEAT)
		switch (key)
		{
		case 'R':
		case 'r':
			std::cout << "SNAKE HEAD LOC: \n" << scn->split_snake[16].GetTranslation() << std::endl;
			std::cout << "OBJECT LOC: \n" << scn->data_list[1].GetTranslation() << std::endl;
			std::cout << "DISTANCE: \n" << (scn->split_snake[16].GetTranslation() - scn->data_list[1].GetTranslation()).norm() << std::endl;
			break;
		case 'A':
		case 'a':
		{
			std::cout << "camera translation: " << rndr->core().camera_translation << std::endl;
			std::cout << "camera eye: " << rndr->core().camera_eye << std::endl;
			std::cout << "camera up: " << rndr->core().camera_up << std::endl;
			//rndr->core().is_animating = !rndr->core().is_animating;
			break;
		}
		case 'F':
		case 'f':
		{
			scn->data().set_face_based(!scn->data().face_based);
			break;
		}
		case 'I':
		case 'i':
		{
			scn->data().dirty |= igl::opengl::MeshGL::DIRTY_NORMAL;
			scn->data().invert_normals = !scn->data().invert_normals;
			break;
		}
		case 'L':
		case 'l':
		{
			rndr->core().toggle(scn->data().show_lines);
			break;
		}
		case 'E':
		case 'e':
		{
			rndr->core().toggle(scn->data().show_overlay);
			break;
		}
		case 'O':
		case 'o':
		{
			rndr->core().orthographic = !rndr->core().orthographic;
			break;
		}
		case '[':
		case ']':
		{
			if (!scn->isTranslated) {
				/*Eigen::Vector3d tempEye = scn->vT[scn->vT.size() - 1];
				rndr->core().camera_eye << tempEye[0], tempEye[1], tempEye[2];
				std::cout << "CAMERA EYE: " << rndr->core().camera_eye << std::endl;
				Eigen::Vector3d tempUp = scn->split_snake[scn->split_snake.size() - 1].GetRotation() * Eigen::Vector3d(0, 1, 0);
				rndr->core().camera_up << tempUp[0], tempUp[1], tempUp[2];
				std::cout << "CAMERA UP: " << rndr->core().camera_up << std::endl;
				std::cout << "UP ROTATION :" << scn->split_snake[scn->split_snake.size() - 1].GetRotation() << std::endl;
				Eigen::Vector3d tempCenter = (scn->split_snake[scn->split_snake.size() - 1].MakeTransd() * Eigen::Vector4d(0, 0, 5, 1)).head(3);
				rndr->core().camera_center << tempCenter[0], tempCenter[1], tempCenter[2];
				std::cout << "CAMERA CENTER: " << rndr->core().camera_center << std::endl;*/
				rndr->core().camera_eye = rndr->core().snake_camera_eye;
				rndr->core().camera_translation = rndr->core().snake_camera_translation;
				scn->isTranslated = true;
			}
			else {
				rndr->core().camera_eye = rndr->core().prev_camera_eye;
				rndr->core().camera_translation = rndr->core().prev_camera_translation;
				//rndr->core().camera_up << 0, 1, 5;
				//rndr->core().camera_center << 0, 0, 0;

				scn->isTranslated = false;
			}


			std::cout << rndr->core().id << std::endl;
			/*rndr->ChangeCamera(key);*/
			break;
		}
		case ';':
			scn->isFog = !scn->isFog;
			break;
		case ':':
			scn->data().show_faceid = !scn->data().show_faceid;
			break;
		case 'w':
		case 'W':
			scn->isPaused = false;
			scn->isActive = true;
			scn->previous_direction = 0;
			scn->direction = 'w';
			break;

		case 's':
		case 'S':
			scn->isPaused = false;
			scn->isActive = true;
			scn->previous_direction = 0;
			scn->direction = 's';
			break;

		case GLFW_KEY_UP:
			scn->isPaused = false;
			scn->isActive = true;
			scn->previous_direction = 0;
			scn->direction = 'u';
			break;

		case GLFW_KEY_DOWN:
			scn->isPaused = false;
			scn->isActive = true;
			scn->previous_direction = 0;
			scn->direction = 'd';
			break;

		case GLFW_KEY_LEFT:
			scn->isPaused = false;
			scn->isActive = true;
			scn->previous_direction = 0;
			scn->direction = 'l';
			break;

		case GLFW_KEY_RIGHT:
			scn->isPaused = false; 
			scn->isActive = true;
			scn->previous_direction = 0;
			scn->direction = 'r';
			break;

		case ' ':
			if (scn->previous_direction) {
				scn->direction = scn->previous_direction;
				scn->previous_direction = 0;
			}
			else {
				scn->previous_direction = scn->direction;
				scn->direction = ' ';
			}
			break;

		default: 
			Eigen::Vector3f shift;
			float scale;
			rndr->core().get_scale_and_shift_to_fit_mesh(scn->data().V, scn->data().F, scale, shift);
			
			std::cout << "near " << rndr->core().camera_dnear << std::endl;
			std::cout << "far " << rndr->core().camera_dfar << std::endl;
			std::cout << "angle " << rndr->core().camera_view_angle << std::endl;
			std::cout << "base_zoom " << rndr->core().camera_base_zoom << std::endl;
			std::cout << "zoom " << rndr->core().camera_zoom << std::endl;
			std::cout << "shift " << shift << std::endl;
			std::cout << "translate " << rndr->core().camera_translation << std::endl;

			break;//do nothing
		}
}


void Init(Display& display, igl::opengl::glfw::imgui::ImGuiMenu *menu)
{
	display.AddKeyCallBack(glfw_key_callback);
	display.AddResizeCallBack(glfw_window_size);
	menu->init(&display);
}
