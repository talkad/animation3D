#include "igl/opengl/glfw/renderer.h"

#include <GLFW/glfw3.h>
#include <igl/unproject_onto_mesh.h>
#include "igl/look_at.h"

//#include <Eigen/Dense>

#define VIEWPORT_WIDTH 1000
#define VIEWPORT_HEIGHT 800

#define e 2.718
#define FOG_START 2
#define FOG_DENSTITY 5



Renderer::Renderer() : selected_core_index(0),
next_core_id(2)
{
	core_list.emplace_back(igl::opengl::ViewerCore());
	core_list.front().id = 1;
	// C-style callbacks
	callback_init = nullptr;
	callback_pre_draw = nullptr;
	callback_post_draw = nullptr;
	callback_mouse_down = nullptr;
	callback_mouse_up = nullptr;
	callback_mouse_move = nullptr;
	callback_mouse_scroll = nullptr;
	callback_key_down = nullptr;
	callback_key_up = nullptr;

	callback_init_data = nullptr;
	callback_pre_draw_data = nullptr;
	callback_post_draw_data = nullptr;
	callback_mouse_down_data = nullptr;
	callback_mouse_up_data = nullptr;
	callback_mouse_move_data = nullptr;
	callback_mouse_scroll_data = nullptr;
	callback_key_down_data = nullptr;
	callback_key_up_data = nullptr;
	//highdpi = 1;

	xold = 0;
	yold = 0;
}

double a = 1; 

IGL_INLINE void Renderer::draw(GLFWwindow* window)
{
	using namespace std;
	using namespace Eigen;

	Eigen::Vector3d colorVec;
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> distr_color(0, 2);
	std::uniform_int_distribution<> distr_sign(0, 1);

	int color;
	int sign;
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);

	int width_window, height_window;
	glfwGetWindowSize(window, &width_window, &height_window);

	auto highdpi_tmp = (width_window == 0 || width == 0) ? highdpi : (width / width_window);

	if (fabs(highdpi_tmp - highdpi) > 1e-8)
	{
		post_resize(window, width, height);
		highdpi = highdpi_tmp;
	}

	for (auto& core : core_list)
	{
		core.clear_framebuffers();
	}
	int coreIndx = 1;
	if (menu)
	{
		menu->pre_draw();
		menu->callback_draw_viewer_menu();
	}

	for (auto& core : core_list)
	{
		int indx = 0;

		for (auto& mesh : scn->data_list)
		{
			if (mesh.is_visible & core.id) {
				{
					if (selected_core_index == 0) {
						//core.camera_translation << -0.5, -0.5, 10;
						//core.camera_eye << -1, 0.25, 0.5;
						//core.camera_up << 0, 0.75, 0;


						Eigen::Matrix4d headTransMat = scn->split_snake[16].MakeTransd();
						//scn->MakeTransd()* scn->CalcSnakeJointsTrans();
						//Eigen::Matrix4d headTransMat = scn->MakeTransd() * scn->CalcParentsTrans(scn->snake_size - 1) * scn->data(scn->snake_size - 1).MakeTransd();
						
						core.camera_translation = (headTransMat * Eigen::Vector4d(-0.5, -0.5, 10, 1)).block(0, 0, 3, 1).cast<float>();
						core.camera_eye = (headTransMat.block(0, 0, 3, 3) * Eigen::Vector3d(-1, 0.25, 0.5)).block(0, 0, 3, 1).cast<float>();
						// core.camera_up = (headTransMat.block(0, 0, 3, 3) * Eigen::Vector3d(0, 0, 0)).block(0, 0, 3, 1).cast<float>();	
						
						//Eigen::Vector3d tempUp = scn->split_snake[16].GetRotation() * Eigen::Vector3d(0, 1, 0);
						//core.camera_up << tempUp[0], tempUp[1], tempUp[2];

						std::cout << "snake head \n" << headTransMat << std::endl;


						//core.camera_up << 0, 5, 5;
						//std::cout << "camera translation: " << core.camera_translation << std::endl;
						//std::cout << "camera eye: " << core.camera_eye << std::endl;
						//std::cout << "camera up: " << core.camera_up << std::endl;

						//core.camera_up << 4, 0, 0;

						//Eigen::Vector3d tempEye = core.camera_translation.cast<double>() - scn->split_snake[16].GetTranslation(); //Eigen::Vector3d(0, 0, 0);
						//core.camera_eye << tempEye[0], tempEye[1], tempEye[2];
						//Eigen::Vector3d tempUp = scn->split_snake[16].GetRotation() * Eigen::Vector3d(0, 1, 0);
						//core.camera_up << tempUp[0], tempUp[1], tempUp[2];
						//Eigen::Vector3d tempCenter = scn->split_snake[16].GetTranslation();
						//core.camera_translation << tempCenter[0], tempCenter[1], tempCenter[2];

						//std::cout << "camera eye: " << core.camera_eye << std::endl;
						//std::cout << "camera up: " << core.camera_up << std::endl;
					}
					else {
						core.camera_translation = prev_camera_translation;
						core.camera_eye = prev_camera_eye;
						core.camera_up = prev_camera_up;

						std::cout << "original camera eye: " << core.camera_eye << std::endl;
						std::cout << "original camera up: " << core.camera_up << std::endl;
						//std::cout << "original camera translation: " << core.camera_translation << std::endl;
					}

					if(!mesh.isTerminated){

						//if (GetScene()->isFog)
						//{	
						//	/*Eigen::Vector3d distanceVector = mesh.GetTranslation() - core.camera_translation.cast <double>();;
						//	double dist = sqrt(distanceVector.dot(distanceVector));*/

						//	double dist = abs(mesh.GetTranslation()[2] - core.camera_translation.cast <double>()[2]); // according z axis

						//	//std::cout << mesh.id << " visibility rate: " << visibility << std::endl;

						//	if (dist > FOG_START && mesh.speed(2) < 0) {
						//		double visibility = -0.161 * dist + 1;

						//		//std::cout << visibility << std::endl;

						//		if(visibility > -1)
						//			mesh.set_colors(RowVector4d(mesh.color(0), mesh.color(1), mesh.color(2), visibility));
						//	}
						//}

						/*if (mesh.id == 1) {
							std::cout << "MESH LOC: \n" << mesh.GetTranslation() << std::endl;
							std::cout << "SNAKE HEAD LOC: \n" << GetScene()->split_snake[15].GetTranslation() << std::endl
								<< "SNAKE HEAD JOINT LOC: \n" << GetScene()->jointBoxes[15].center() << std::endl;
						}*/

						if (mesh.type == 4) {
							color = distr_color(gen);
							sign = distr_sign(gen);

							sign == 0 ? sign = 1 : sign = -1;

							switch (color) {
							case 1:
								colorVec = mesh.color + sign * Eigen::Vector3d(1, 0, 0) * 0.1;
								break;
							case 2:
								colorVec = mesh.color + sign * Eigen::Vector3d(0, 1, 0) * 0.1;
								break;
							default:
								colorVec = mesh.color + sign * Eigen::Vector3d(0, 0, 1) * 0.05;
								break;
							}

							mesh.set_colors(Eigen::RowVector3d(colorVec(0), colorVec(1), colorVec(2)));

						}


						core.draw(scn->MakeTransScale() * scn->CalcParentsTrans(indx).cast<float>(), mesh);
					}

				}
			}
			indx++;
		}
	}

	if (menu)
	{
		menu->post_draw();
	}

}



void Renderer::SetScene(igl::opengl::glfw::Viewer* viewer)
{
	scn = viewer;
}

IGL_INLINE void Renderer::init(igl::opengl::glfw::Viewer* viewer, int coresNum, igl::opengl::glfw::imgui::ImGuiMenu* _menu)
{
	scn = viewer;

	doubleVariable = 0;
	core().init();
	menu = _menu;
	core().align_camera_center(scn->data().V, scn->data().F);

	if (coresNum > 1)
	{
		core().viewport = Eigen::Vector4f(0, 0, VIEWPORT_WIDTH , VIEWPORT_HEIGHT);
		left_view = core_list[0].id;
		right_view = append_core(Eigen::Vector4f(VIEWPORT_WIDTH / 4, 0, VIEWPORT_WIDTH * 3 / 4, VIEWPORT_HEIGHT));
		core_index(right_view - 1);
		for (size_t i = 0; i < scn->data_list.size(); i++)
		{
			core().toggle(scn->data_list[i].show_faces);
			core().toggle(scn->data_list[i].show_lines);
			core().toggle(scn->data_list[i].show_texture);
		}
		//Eigen::Vector3d v = -scn->GetCameraPosition();
		//TranslateCamera(v.cast<float>());

		core_index(left_view - 1);

		prev_camera_translation = core().camera_translation;
		prev_camera_eye = core().camera_eye;
		prev_camera_up = core().camera_up;

	}

	//selected_core_index = 0;

	if (menu)
	{
		menu->callback_draw_viewer_menu = [&]()
		{
			// Draw parent menu content
			//menu->draw_viewer_menu(scn, core_list);
			menu->callback_draw_custom_window();
			
		};
	}
}

void Renderer::UpdatePosition(double xpos, double ypos)
{
	xrel = xold - xpos;
	yrel = yold - ypos;
	xold = xpos;
	yold = ypos;
}

void Renderer::MouseProcessing(int button)
{

	if (scn->isPicked)
	{
		if (button == 1)
		{
			float near = core().camera_dnear, far = core().camera_dfar, angle = core().camera_view_angle;
			//float z = far + depth * (near - far);

			Eigen::Matrix4f tmpM = core().proj;
			double xToMove = -(double)xrel / core().viewport[3] * (z + 2 * near) * (far) / (far + 2 * near) * 2.0 * tanf(angle / 360 * M_PI) / (core().camera_zoom * core().camera_base_zoom);
			double yToMove = (double)yrel / core().viewport[3] * (z + 2 * near) * (far) / (far + 2 * near) * 2.0 * tanf(angle / 360 * M_PI) / (core().camera_zoom * core().camera_base_zoom);

			if (scn->selected_data_index == 0) {
				scn->data().MyTranslateInSystem(scn->GetRotation(), Eigen::Vector3d(xToMove, 0, 0));
				scn->data().MyTranslateInSystem(scn->GetRotation(), Eigen::Vector3d(0, yToMove, 0));
			}
			else {
				scn->data_list[1].MyTranslateInSystem(scn->GetRotation(), Eigen::Vector3d(xToMove, 0, 0));
				scn->data_list[1].MyTranslateInSystem(scn->GetRotation(), Eigen::Vector3d(0, yToMove, 0));
			}

			scn->WhenTranslate();
		}
		else
		{
			scn->data().RotateInSystem(Eigen::Vector3d(1, 0, 0), yrel / 180.0);
			scn->data().RotateInSystem(Eigen::Vector3d(0, 1, 0), xrel / 180.0);

		}
	}
	else
	{
		if (button == 1)
		{
			float near = core().camera_dnear, far = core().camera_dfar, angle = core().camera_view_angle;
			float z = far + 0.5f * (near - far);


			double xToMove = -(double)xrel / core().viewport[3] * far / z * near * 2.0f * tanf(angle / 360 * M_PI) / (core().camera_zoom * core().camera_base_zoom);
			double yToMove = (double)yrel / core().viewport[3] * far / z * near * 2.0f * tanf(angle / 360 * M_PI) / (core().camera_zoom * core().camera_base_zoom);

			scn->MyTranslate(Eigen::Vector3d(xToMove, 0, 0), true);
			scn->MyTranslate(Eigen::Vector3d(0, yToMove, 0), true);

		}
		else
		{
			scn->MyRotate(Eigen::Vector3d(1, 0, 0), yrel / 180.0);
			scn->MyRotate(Eigen::Vector3d(0, 1, 0), xrel / 180.0);
		}
	}
}

void Renderer::TranslateCamera(Eigen::Vector3f amt)
{
	core().camera_translation += amt;
}

void Renderer::RotateCamera(float amtX, float amtY)
{
	core().camera_eye = core().camera_eye + Eigen::Vector3f(0, amtY, 0);
	Eigen::Matrix3f Mat;
	Mat << cos(amtY), 0, sin(amtY), 0, 1, 0, -sin(amtY), 0, cos(amtY);
	core().camera_eye = Mat * core().camera_eye;

}


Renderer::~Renderer()
{
	//if (scn)
	//	delete scn;
}

double Renderer::Picking(double newx, double newy)
{	
	int fid;
	//Eigen::MatrixXd C = Eigen::MatrixXd::Constant(scn->data().F.rows(), 3, 1);
	Eigen::Vector3f bc;
	double x = newx;
	double y = core().viewport(3) - newy;

	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

	igl::look_at(core().camera_eye, core().camera_center, core().camera_up, view);
	//std::cout << "view matrix\n" << view << std::endl;
	view = view * (core().trackball_angle * Eigen::Scaling(core().camera_zoom * core().camera_base_zoom)
		* Eigen::Translation3f(core().camera_translation + core().camera_base_translation)).matrix() * scn->MakeTransScale() * scn->CalcParentsTrans(scn->selected_data_index).cast<float>() * scn->data().MakeTransScale();
	bool picked = igl::unproject_onto_mesh(Eigen::Vector2f(x, y), view,
		core().proj, core().viewport, scn->data().V, scn->data().F, fid, bc);
	scn->isPicked = scn->isPicked | picked;
	if (picked)
	{
		Eigen::Vector3i face = scn->data().F.row(fid);
		Eigen::Matrix3d vertices;
		Eigen::Vector4f p, pp;

		vertices.col(0) = scn->data().V.row(face(0));
		vertices.col(1) = scn->data().V.row(face(1));
		vertices.col(2) = scn->data().V.row(face(2));

		p << vertices.cast<float>() * bc, 1;
		p = view * p;
		//std::cout << scn->data().V.row(face(0)) << std::endl;
		pp = core().proj * p;
		//glReadPixels(x,  y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
		z = pp(2);
		return p(2);

	}
	return 0;
}

IGL_INLINE void Renderer::resize(GLFWwindow* window, int w, int h)
{
	if (window) {
		glfwSetWindowSize(window, w / highdpi, h / highdpi);
	}
	post_resize(window, w, h);
}

IGL_INLINE void Renderer::post_resize(GLFWwindow* window, int w, int h)
{
	if (core_list.size() == 1)
	{

		core().viewport = Eigen::Vector4f(0, 0, w, h);
	}
	else
	{
		// It is up to the user to define the behavior of the post_resize() function
		// when there are multiple viewports (through the `callback_post_resize` callback)
		core(left_view).viewport = Eigen::Vector4f(0, 0, w / 4, h);
		core(right_view).viewport = Eigen::Vector4f(w / 4, 0, w - (w / 4), h);

	}
	//for (unsigned int i = 0; i < plugins.size(); ++i)
	//{
	//	plugins[i]->post_resize(w, h);
	//}
	if (callback_post_resize)
	{
		callback_post_resize(window, w, h);
	}
}

IGL_INLINE igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/)
{
	assert(!core_list.empty() && "core_list should never be empty");
	int core_index;
	if (core_id == 0)
		core_index = selected_core_index;
	else
		core_index = this->core_index(core_id);
	assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
	return core_list[core_index];
}

IGL_INLINE const igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/) const
{
	assert(!core_list.empty() && "core_list should never be empty");
	int core_index;
	if (core_id == 0)
		core_index = selected_core_index;
	else
		core_index = this->core_index(core_id);
	assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
	return core_list[core_index];
}

IGL_INLINE bool Renderer::erase_core(const size_t index)
{
	assert((index >= 0 && index < core_list.size()) && "index should be in bounds");
	//assert(data_list.size() >= 1);
	if (core_list.size() == 1)
	{
		// Cannot remove last viewport
		return false;
	}
	core_list[index].shut(); // does nothing
	core_list.erase(core_list.begin() + index);
	if (selected_core_index >= index && selected_core_index > 0)
	{
		selected_core_index--;
	}
	return true;
}

IGL_INLINE size_t Renderer::core_index(const int id) const {
	for (size_t i = 0; i < core_list.size(); ++i)
	{
		if (core_list[i].id == id)
			return i;
	}
	return 0;
}

IGL_INLINE int Renderer::append_core(Eigen::Vector4f viewport, bool append_empty /*= false*/)
{
	core_list.push_back(core()); // copies the previous active core and only changes the viewport
	core_list.back().viewport = viewport;
	core_list.back().id = next_core_id;
	next_core_id <<= 1;
	if (!append_empty)
	{
		for (auto& data : scn->data_list)
		{
			data.set_visible(true, core_list.back().id);
			//data.copy_options(core(), core_list.back());
		}
	}
	selected_core_index = core_list.size() - 1;
	return core_list.back().id;
}