#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"

class SandBox : public igl::opengl::glfw::Viewer
{
public:
	bool isTranslated;
	SandBox();
	~SandBox();
	void Init(const std::string& config);

private:	
	void Animate();
};

