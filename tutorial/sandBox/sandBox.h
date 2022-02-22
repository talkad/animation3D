#pragma once
#include "igl/opengl/glfw/Viewer.h"

class SandBox : public igl::opengl::glfw::Viewer
{
public:
	SandBox();
	~SandBox();
	void Init(const std::string& config);

private:	
	void Animate();
};

