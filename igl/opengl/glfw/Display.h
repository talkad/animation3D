#pragma once
#include <string>
#include <GLFW/glfw3.h>
#include <external/glm/glm.hpp>

//#include "igl/opengl/glfw/renderer.h"
#define EXIT_FAILURE 1
struct GLFWwindow;

// Represents a single particle and its state
struct Particle {
	glm::vec2 Position, Velocity;
	glm::vec4 Color;
	float     Life;

	Particle() : Position(0.0f), Velocity(0.0f), Color(1.0f), Life(0.0f) { }
};

// ParticleGenerator acts as a container for rendering a large number of 
// particles by repeatedly spawning and updating particles and killing 
// them after a given amount of time.
class ParticleGenerator
{
public:
	// constructor
	ParticleGenerator(unsigned int amount);
	// update all particles
	void Update(float dt, unsigned int newParticles, glm::vec2 offset = glm::vec2(0.0f, 0.0f));
	// render all particles
	void Draw();

	// state
	unsigned int lastUsedParticle;
	std::vector<Particle> particles;
	unsigned int amount;
	// render state
	unsigned int VAO;
	// initializes buffer and vertex attributes
	void init();
	// returns the first Particle index that's currently unused e.g. Life <= 0.0f or 0 if no particle is currently inactive
	unsigned int firstUnusedParticle();
	// respawns particle
	void respawnParticle(Particle &particle, glm::vec2 offset = glm::vec2(0.0f, 0.0f));
};


class Display
{
public:
	Display(int windowWidth, int windowHeight, const std::string& title);
	
	bool launch_rendering(bool loop);

	void SwapBuffers();
	void PollEvents();

	void SetRenderer(void* userPointer);
	void* GetScene();
	void AddKeyCallBack(void(*func)(GLFWwindow*, int, int, int, int));
	void AddMouseCallBacks(void (*mousebuttonfun)(GLFWwindow*, int, int, int), void(*scrollfun)(GLFWwindow*, double, double), void (*cursorposfun)(GLFWwindow*, double, double));
	void AddResizeCallBack(void (*windowsizefun)(GLFWwindow*, int, int));

	
	~Display();

//private:
	GLFWwindow* window;
	void* renderer;
	ParticleGenerator* particleGen;
	//int highdpi;  //relation between width and height?

};