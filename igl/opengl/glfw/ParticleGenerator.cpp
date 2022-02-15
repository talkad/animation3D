
#include <igl/opengl/glfw/ParticleGenerator.h>
#include <glad\glad.h>

ParticleGenerator::ParticleGenerator(unsigned int amount): amount(amount), lastUsedParticle(0)
{
	this->init();
}

void ParticleGenerator::Update(float dt, unsigned int newParticles, glm::vec2 offset)
{
	 // add new particles 
	 for (unsigned int i = 0; i < newParticles; ++i)
	 {
	     int unusedParticle = this->firstUnusedParticle();
	     this->respawnParticle(this->particles[unusedParticle], offset);
	 }
	 // update all particles
	 for (unsigned int i = 0; i < this->amount; ++i)
	 {
	     Particle &p = this->particles[i];
	     p.Life -= dt; // reduce life
	     if (p.Life > 0.0f)
	     {	// particle is alive, thus update
	         p.Position -= p.Velocity * dt; 
	         p.Color.a -= dt * 2.5f;
	     }
	 }
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

void ParticleGenerator::respawnParticle(Particle& particle, glm::vec2 offset)
{
	float random = ((rand() % 100) - 50) / 10.0f;
	float rColor = 0.5f + ((rand() % 100) / 100.0f);
	//particle.Position = object.Position + random + offset;
	particle.Color = glm::vec4(rColor, rColor, rColor, 1.0f);
	particle.Life = 1.0f;
	//particle.Velocity = object.Velocity * 0.1f;
}
