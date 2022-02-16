
#include <igl/opengl/glfw/ParticleGenerator.h>
#include <glad\glad.h>
#include <iostream>


ParticleGenerator::ParticleGenerator(unsigned int amount, bool isExplosion, double x, double y):
amount(amount), lastUsedParticle(0), camera_angle(front), isExplosion(isExplosion), x(x), y(y)
{
	this->init();
}

void ParticleGenerator::Update(float dt, unsigned int newParticles)
{
	 // add new particles 
	 for (unsigned int i = 0; i < newParticles; ++i)
	 {
	     int unusedParticle = this->firstUnusedParticle();
	     this->respawnParticle(this->particles[unusedParticle]);
	 }

	 // update all particles
	 for (unsigned int i = 0; i < this->amount; ++i)
	 {
	     Particle &p = this->particles[i];
	     p.Life -= dt; // reduce life

		 if (p.Life > 0.0f) // particle is alive, thus update
		 {	
			 // move in random dir
			 float x_dir_rand = randRange(0, 100) / 100.0f;
			 float y_dir_rand = randRange(0, 100) / 100.0f;

			 p.Position -= glm::vec2(p.Velocity[0] * x_dir_rand, p.Velocity[1] * y_dir_rand) * dt;
			 p.Color.a -= dt * 0.1f;
		 }
	 }
}

void ParticleGenerator::set_camera_angle(int dir)
{
	dir == 0 ? camera_angle = down :
		dir == 1 ? camera_angle = semi_down :
		dir == 2 ? camera_angle = front :
		dir == 3 ? camera_angle = above_front :
		dir == 4 ? camera_angle = semi_up :  camera_angle = up;
}

void ParticleGenerator::init()
{
	// set up mesh and attribute properties
	unsigned int VBO;
	float particle_quad[] = {
		0.0f, 1.0f, 0.0f, 1.0f,
		1.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f,

		0.0f, 1.0f, 0.0f, 1.0f,
		1.0f, 1.0f, 1.0f, 1.0f,
		1.0f, 0.0f, 1.0f, 0.0f
	};
	glGenVertexArrays(1, &this->VAO);
	glGenBuffers(1, &VBO);
	glBindVertexArray(this->VAO);
	// fill mesh buffer
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(particle_quad), particle_quad, GL_STATIC_DRAW);
	// set mesh attributes
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
	glBindVertexArray(0);

	// create this->amount default particle instances
	for (unsigned int i = 0; i < this->amount; ++i)
		this->particles.push_back(Particle());
}

unsigned int ParticleGenerator::firstUnusedParticle()
{
	// first search from last used particle, this will usually return almost instantly
	for (unsigned int i = lastUsedParticle; i < this->amount; ++i) {
		if (this->particles[i].Life <= 0.0f) {
			lastUsedParticle = i;
			return i;
		}
	}
	// otherwise, do a linear search
	for (unsigned int i = 0; i < lastUsedParticle; ++i) {
		if (this->particles[i].Life <= 0.0f) {
			lastUsedParticle = i;
			return i;
		}
	}
	// all particles are taken, override the first one (note that if it repeatedly hits this case, more particles should be reserved)
	lastUsedParticle = 0;
	return 0;
}

void ParticleGenerator::respawnParticle(Particle& particle)
{
	// move in random dir
	float x_dir_rand = (randRange(0, 200) - 100) / 100.0f;
	float y_dir_rand = (randRange(0, 200) - 100) / 100.0f;

	if (isExplosion) {
		particle.Color = glm::vec4(0.0f, 0.0f, 100.0f, 1.0f);
		particle.Position = glm::vec2(x, y);
		particle.Velocity = glm::vec2(100 * x_dir_rand, 100 * y_dir_rand) * 0.3f;
	}
	else {
		float x_pos_rand = randRange(0, 1000);
		float y_pos_rand = randRange(0, camera_angle);

		int color_rand = randRange(0, 2);

		switch (color_rand) {
			case 0:
				particle.Color = glm::vec4(75.0f, 75.0f, 75.0f, 1.0f);
				break;
			case 1:
				particle.Color = glm::vec4(0.0f, 75.0f, 0.0f, 1.0f);
				break;
			case 2:
				particle.Color = glm::vec4(0.0f, 0.0f, 75.0f, 1.0f);
				break;
		}

		particle.Position =  glm::vec2(x_pos_rand, y_pos_rand);
		particle.Velocity = glm::vec2(200 * x_dir_rand, 100 * y_dir_rand) * 0.1f;
	}

	particle.Life = 10.0f;
	
}

float ParticleGenerator::randRange(int min, int max)
{
	std::mt19937 gen(rd()); // seed the generator
	std::uniform_int_distribution<> distr_dir(min, max); // define the range

	return distr_dir(gen);
}
