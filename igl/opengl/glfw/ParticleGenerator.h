#include <vector>
#include <random>
#include <external/glad/include/glad/glad.h>
#include <external/glm/glm.hpp>

// Represents a single particle and its state
struct Particle {
	glm::vec2 Position, Velocity;
	glm::vec4 Color;
	float     Life;

	Particle() : Position(0.0f), Velocity(0.0f), Color(1.0f), Life(0.0f) { }
};

enum CameraAngle {
	down = 0,
	semi_down = 200,
	front = 300,
	above_front = 500,
	semi_up = 650,
	up = 800
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
	void Update(float dt, unsigned int newParticles);
	void set_camera_angle(int);

	// state
	unsigned int lastUsedParticle;
	std::vector<Particle> particles;
	unsigned int amount;
	// render state
	unsigned int VAO;

	

private:
	std::random_device rd; // obtain a random number from hardware
	CameraAngle camera_angle;

	// initializes buffer and vertex attributes
	void init();
	// returns the first Particle index that's currently unused e.g. Life <= 0.0f or 0 if no particle is currently inactive
	unsigned int firstUnusedParticle();
	// respawns particle
	void respawnParticle(Particle& particle);

	float randRange(int min, int max);
};
