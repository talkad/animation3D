#pragma once
#include "igl/opengl/glfw/Display.h"
#include "igl/opengl/glfw/Renderer.h"
#include "sandBox.h"
#include "igl/look_at.h"
//#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
//#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
//#include <../imgui/imgui.h>



static void glfw_mouse_press(GLFWwindow* window, int button, int action, int modifier)
{

  Renderer* rndr = (Renderer*) glfwGetWindowUserPointer(window);
  igl::opengl::glfw::Viewer* scn = rndr->GetScene();

  if (action == GLFW_PRESS)
  {
	  double x2, y2;
	  glfwGetCursorPos(window, &x2, &y2);
	 

	  double depth, closestZ = 1;
	  int i = 0, savedIndx = scn->selected_data_index, lastIndx= scn->selected_data_index;

	  int prev_picked = scn->current_picked;

	  for (; i < scn->data_list.size(); i++)
	  {
		  scn->selected_data_index = i;
		  depth = rndr->Picking(x2, y2);
		  if (depth < 0 && (closestZ > 0 || closestZ < depth))
		  {
			  scn->current_picked = i;
			  savedIndx = i;
			  closestZ = depth;
			  std::cout << "--- found " << depth << std::endl;
		  }
	  }
	  scn->selected_data_index = savedIndx;
	  scn->data().set_colors(Eigen::RowVector3d(0.2, 0.7, 0.8));
	  if (lastIndx != savedIndx)
		  scn->data_list[lastIndx].set_colors(Eigen::RowVector3d(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0));

	  if (scn->current_picked == prev_picked) {
		  scn->current_picked = -1;
	  }

	  rndr->UpdatePosition(x2, y2);
  }
  else
  {
	  rndr->GetScene()->isPicked = false;
  }
}


//static void glfw_char_mods_callback(GLFWwindow* window, unsigned int codepoint, int modifier)
//{
//  __viewer->key_pressed(codepoint, modifier);
//}

 void glfw_mouse_move(GLFWwindow* window, double x, double y)
{
	 Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	 rndr->UpdatePosition(x, y);
	 if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
	 {
		 rndr->MouseProcessing(GLFW_MOUSE_BUTTON_RIGHT);
	 }
	 else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
	 {
		 rndr->MouseProcessing(GLFW_MOUSE_BUTTON_LEFT);
	 }
}

 static void glfw_mouse_scroll(GLFWwindow* window, double x, double y)
 {
	 Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	 if (rndr->IsPicked())
		 if (rndr->GetScene()->selected_data_index == 0)
			rndr->GetScene()->data().MyTranslateInSystem(rndr->GetScene()->GetRotation(), Eigen::Vector3d(0, 0, y));
		 else
			rndr->GetScene()->data_list[1].MyTranslateInSystem(rndr->GetScene()->GetRotation(), Eigen::Vector3d(0, 0, y));
	 //rndr->GetScene()->data_list[1].MyScale(Eigen::Vector3d(1 + y * 0.01,1 + y * 0.01,1+y*0.01));
	 else
		 rndr->GetScene()->MyTranslate(Eigen::Vector3d(0, 0, -y * 0.03), true);
 }

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
		case 'T':
		case 't':
		{
			if(scn->link_num > 0)
				std::cout << "tip: (" << scn->calcJointPos(scn->link_num + 1) << ")\n" << std::endl;
			break;
		}
		case 'D':
		case 'd':
		{
			if (scn->link_num > 0) {
				scn->destination = scn->data(0).MakeTransd().col(3).head(3);
				std::cout << "destination: (" << scn->destination.transpose() << ")\n" << std::endl;
			}
			break;
		}
		case 'P':
		case 'p':
		{
			Eigen::Matrix3d mat;
			int idx = scn->current_picked;

			if (scn->current_picked != -1)
			{
				mat = scn->data_list[idx].GetRotation();
				std::cout << "rotation of link " << idx << ":\n" << std::endl;
			}
			else
			{
				mat = scn->GetRotation();
				std::cout << "rotation of scene:\n" << std::endl;
			}
			
			std::cout << mat << "\n" <<std::endl;
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
		case ' ':
			// toggle ik solver aniimation
			if (scn->data_list.size() > 1) {
				Eigen::Vector4d root = scn->data_list[1].MakeTransd() * Eigen::Vector4d(0, 0, -scn->link_length / 2, 1);
				Eigen::Vector4d ball = scn->data_list[0].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1);

				double dist = (root - ball).norm();

				if (scn->link_num * scn->link_length >= dist)
					scn->SetAnimation();
				else {
					std::cout << "cannot reach" << std::endl;
					scn->isActive = false;
				}
			}
			else {
				std::cout << "cannot reach" << std::endl;
				scn->isActive = false;
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
	display.AddMouseCallBacks(glfw_mouse_press, glfw_mouse_scroll, glfw_mouse_move);
	display.AddResizeCallBack(glfw_window_size);
	menu->init(&display);
}
