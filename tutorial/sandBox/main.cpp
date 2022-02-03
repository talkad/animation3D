#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "sandBox.h"

#define VIEWPORT_WIDTH 1000
#define VIEWPORT_HEIGHT 800

int main(int argc, char* argv[])
{
	Display* disp = new Display(VIEWPORT_WIDTH, VIEWPORT_HEIGHT, "Animation3D - Final Project");
	Renderer renderer;

	SandBox viewer;

	igl::opengl::glfw::imgui::ImGuiMenu* menu = new igl::opengl::glfw::imgui::ImGuiMenu();
	viewer.Init("configuration.txt");

	Init(*disp, menu);
	renderer.init(&viewer, 2, menu);

	disp->SetRenderer(&renderer);
	disp->launch_rendering(true);
	delete menu;
	delete disp;
}