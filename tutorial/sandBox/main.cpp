#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "sandBox.h"
#include <Windows.h>
#include <MMSystem.h>
#pragma comment(lib, "winmm.lib")

#define VIEWPORT_WIDTH 3840
#define VIEWPORT_HEIGHT 2160



int main(int argc, char* argv[])
{
	//PlaySound(TEXT("C:/Users/tal74/projects/animation/animation3D/tutorial/sandBox/applause.wav"), NULL, SND_NODEFAULT | SND_LOOP | SND_ASYNC);


	Display* disp = new Display(VIEWPORT_WIDTH, VIEWPORT_HEIGHT, "Animation3D - Final Project");
	Renderer renderer;

	SandBox viewer;

	igl::opengl::glfw::imgui::ImGuiMenu menu;
	viewer.Init("configuration.txt");
	Init(*disp, &menu);
	renderer.init(&viewer, 2, &menu);

	disp->SetRenderer(&renderer);
	menu.init_callback(viewer);

	disp->launch_rendering(true);


	delete disp;
}