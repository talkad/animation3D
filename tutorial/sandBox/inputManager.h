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
		case 'A':
		case 'a':
		{
			rndr->core().is_animating = !rndr->core().is_animating;
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
			rndr->ChangeCamera(key);
			break;
		}
		case ';':
			scn->data().show_vertid = !scn->data().show_vertid;
			break;
		case ':':
			scn->data().show_faceid = !scn->data().show_faceid;
			break;
		case 'w':
		case 'W':
			rndr->RotateCamera(0, 0.05f);
			//rndr->TranslateCamera(Eigen::Vector3f(0, 0, 0.03f));
			break;
		case 's':
		case 'S':
			rndr->RotateCamera(0, -0.05f);
			//rndr->TranslateCamera(Eigen::Vector3f(0, 0, -0.03f));
			break;
		case GLFW_KEY_UP:
			if (scn->current_picked != -1)
				scn->data().MyRotate(Eigen::Vector3d(1, 0, 0), 0.1);
			else
				scn->MyRotate(Eigen::Vector3d(1, 0, 0), 0.1);

			//rndr->TranslateCamera(Eigen::Vector3f(0, 0.01f,0));
			break;
		case GLFW_KEY_DOWN:
			if (scn->current_picked != -1)
				scn->data().MyRotate(Eigen::Vector3d(1, 0, 0), -0.1);
			else
				scn->MyRotate(Eigen::Vector3d(1, 0, 0), -0.1);

			//rndr->TranslateCamera(Eigen::Vector3f(0, -0.01f,0));

			break;

		case GLFW_KEY_LEFT:
			if (scn->current_picked != -1)
				scn->data().MyRotate(Eigen::Vector3d(0, 0, 1), -0.1);
			else
				scn->MyRotate(Eigen::Vector3d(0, 0, 1), -0.1);

				//rndr->TranslateCamera(Eigen::Vector3f(-0.01f, 0,0));
			break;

		case GLFW_KEY_RIGHT:
			if (scn->current_picked != -1)
				scn->data().MyRotate(Eigen::Vector3d(0, 0, 1), 0.1);
			else
				scn->MyRotate(Eigen::Vector3d(0, 0, 1), 0.1);

			//rndr->TranslateCamera(Eigen::Vector3f(0.01f, 0, 0));
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
