#include <chrono>
#include <thread>

#include "../gl.h"
#include "Display.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <external/stb_image.h>

#include <external/glm/glm.hpp>
#include <external/glm/gtc/matrix_transform.hpp>
#include <external/glm/gtc/type_ptr.hpp>

#include "igl/igl_inline.h"
#include <igl/get_seconds.h>
#include "igl/opengl/glfw/renderer.h"

#define VIEWPORT_WIDTH 1000
#define VIEWPORT_HEIGHT 800

#include <external/learnopengl/filesystem.h>
#include <external/learnopengl/shader_m.h>
#include <external/learnopengl/camera.h>
#include <external/learnopengl/model.h>

#include "texture.h"

#include <Windows.h>
#include <MMSystem.h>
#pragma comment(lib, "winmm.lib")

ParticleGenerator* explosion;
double last_explosion_time;


void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, int button, int action, int modifier);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void mouse_move(GLFWwindow* window, double x, double y);
void processInput(GLFWwindow* window);

Texture2D loadTexture(const char* path, bool alpha);
unsigned int loadCubemap(vector<std::string> faces);

const unsigned int SCR_WIDTH = 1000;
const unsigned int SCR_HEIGHT = 800;

// camera
Camera camera(glm::vec3(67.0f, 627.5f, 169.9f),
	glm::vec3(0.0f, 1.0f, 0.0f),
	-128.1f, -42.4f);
//Camera camera(glm::vec3(0.0f, 10.0f, 3.0f));
float lastX = (float)SCR_WIDTH / 2.0;
float lastY = (float)SCR_HEIGHT / 2.0;
bool firstMouse = true;


// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;


static void glfw_error_callback(int error, const char* description)
{
	fputs(description, stderr);
}

Display::Display(int windowWidth, int windowHeight, const std::string& title): renderer(0)
{
	bool resizable = true, fullscreen = false;
	glfwSetErrorCallback(glfw_error_callback);
	if (!glfwInit())
	{
		exit(EXIT_FAILURE);
	}
	glfwWindowHint(GLFW_SAMPLES, 8);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

	//#ifdef __APPLE__
	//		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	//		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	//#endif
	//		if (fullscreen)
	//		{
	//			GLFWmonitor* monitor = glfwGetPrimaryMonitor();
	//			const GLFWvidmode* mode = glfwGetVideoMode(monitor);
	//			window = glfwCreateWindow(mode->width, mode->height, title.c_str(), monitor, nullptr);
	//			windowWidth = mode->width;
	//			windowHeight = mode->height;
	//		}
	//		else
	//		{
				// Set default windows width
				//if (windowWidth <= 0 & core_list.size() == 1 && renderer->core().viewport[2] > 0)
				//	windowWidth = renderer->core().viewport[2];
				//else 
				//	if (windowWidth <= 0)
				//	windowWidth = 1280;
				//// Set default windows height
				//if (windowHeight <= 0 & core_list.size() == 1 && renderer->core().viewport[3] > 0)
				//	windowHeight = renderer->core().viewport[3];
				//else if (windowHeight <= 0)
				//	windowHeight = 800;
	//			window = glfwCreateWindow(windowWidth, windowHeight, title.c_str(), nullptr, nullptr);
	//		}
	// 
	window = glfwCreateWindow(windowWidth, windowHeight, title.c_str(), glfwGetPrimaryMonitor(), nullptr);
	//window = glfwCreateWindow(windowWidth, windowHeight, title.c_str(), nullptr, nullptr);
	if (!window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetMouseButtonCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetCursorPosCallback(window, mouse_move);

	// tell GLFW to capture our mouse
	// glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED); 

	// Load OpenGL and its extensions
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		printf("Failed to load OpenGL and its extensions\n");
		exit(EXIT_FAILURE);
	}
	//#if defined(DEBUG) || defined(_DEBUG)
	//		printf("OpenGL Version %d.%d loaded\n", GLVersion.major, GLVersion.minor);
	//		int major, minor, rev;
	//		major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
	//		minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
	//		rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
	//		printf("OpenGL version received: %d.%d.%d\n", major, minor, rev);
	//		printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
	//		printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));
	//#endif

	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

	//Tamir: changes from here
	// Initialize FormScreen
   // __viewer = this;
	// Register callbacks
	//glfwSetKeyCallback(window, glfw_key_callback);
	//glfwSetCursorPosCallback(window,glfw_mouse_move);
	//glfwSetScrollCallback(window, glfw_mouse_scroll);
	//glfwSetMouseButtonCallback(window, glfw_mouse_press);
	//glfwSetWindowSizeCallback(window,glfw_window_size);


	//glfwSetCharModsCallback(window,glfw_char_mods_callback);
	//glfwSetDropCallback(window,glfw_drop_callback);
	// Handle retina displays (windows and mac)
	//int width, height;
	//glfwGetFramebufferSize(window, &width, &height);
	//int width_window, height_window;
	//glfwGetWindowSize(window, &width_window, &height_window);
	//highdpi = windowWidth/width_window;

	//glfw_window_size(window,width_window,height_window);
	//opengl.init();
//		core().align_camera_center(data().V, data().F);
		// Initialize IGL viewer
//		init();

}

bool Display::launch_rendering(bool loop)
{
	// glfwMakeContextCurrent(window);
	float skyboxVertices[] = {
		// positions          
		-1.0f,  1.0f, -1.0f,
		-1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		-1.0f,  1.0f, -1.0f,
		 1.0f,  1.0f, -1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		 1.0f, -1.0f,  1.0f
	};

	Shader skyboxShader("../../../shaders/skybox.vs", "../../../shaders/skybox.fs");
	// set up vertex data (and buffer(s)) and configure vertex attributes
	// ------------------------------------------------------------------
	// skybox VAO
	unsigned int skyboxVAO, skyboxVBO;
	glGenVertexArrays(1, &skyboxVAO);
	glGenBuffers(1, &skyboxVBO);
	glBindVertexArray(skyboxVAO);
	glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	// load textures
	// -------------
	vector<std::string> faces
	{
		FileSystem::getPath("tutorial/textures/skybox/right.jpg"),
		FileSystem::getPath("tutorial/textures/skybox/left.jpg"),
		FileSystem::getPath("tutorial/textures/skybox/top.jpg"),
		FileSystem::getPath("tutorial/textures/skybox/bottom.jpg"),
		FileSystem::getPath("tutorial/textures/skybox/front.jpg"),
		FileSystem::getPath("tutorial/textures/skybox/back.jpg")

	};

	unsigned int cubemapTexture = loadCubemap(faces);

	skyboxShader.use(); // shader configuration
	skyboxShader.setInt("skybox", 0);

	// fog
	Shader fogShader("../../../shaders/particle.vs", "../../../shaders/particle.fs");
	glm::mat4 projection = glm::ortho(0.0f, static_cast<float>(VIEWPORT_WIDTH),
		static_cast<float>(VIEWPORT_HEIGHT), 0.0f, -1.0f, 1.0f);
	
	fogShader.use();
	fogShader.setInt("sprite", 0);
	fogShader.setMat4("projection", projection);

	Texture2D fogTexture = loadTexture("../../../tutorial/textures/particle.png", true);
	particleGen = new ParticleGenerator(2000);
	explosion = nullptr;


	//heightmap

	// configure global opengl state
	// -----------------------------
	glEnable(GL_DEPTH_TEST);

	// build and compile our shader program
	// ------------------------------------
	Shader heightMapShader("../../../shaders/cpuheight.vs", "../../../shaders/cpuheight.fs");

	// load and create a texture
	// -------------------------
	// load image, create texture and generate mipmaps
	// The FileSystem::getPath(...) is part of the GitHub repository so we can find files on any IDE/platform; replace it with your own image path.
	stbi_set_flip_vertically_on_load(true);
	int width, height, nrChannels;
	unsigned char* data = stbi_load("../../../tutorial/heightmaps/river_heightmap.png", &width, &height, &nrChannels, 0);

	// set up vertex data (and buffer(s)) and configure vertex attributes
	// ------------------------------------------------------------------
	std::vector<float> vertices;
	float yScale = 64.0f / 256.0f, yShift = 50.0f;
	int rez = 1;
	unsigned bytePerPixel = nrChannels;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			unsigned char* pixelOffset = data + (j + width * i) * bytePerPixel;
			unsigned char y = pixelOffset[0];

			// vertex
			vertices.push_back(-height / 2.0f + height * i / (float)height);   // vx
			vertices.push_back((int)y * yScale - yShift);   // vy
			vertices.push_back(-width / 2.0f + width * j / (float)width);   // vz
		}
	}
	stbi_image_free(data);

	std::vector<unsigned> indices;
	for (unsigned i = 0; i < height - 1; i += rez)	
		for (unsigned j = 0; j < width; j += rez)		
			for (unsigned k = 0; k < 2; k++)			
				indices.push_back(j + width * (i + k * rez));
			
		
	const int numStrips = (height - 1) / rez;
	const int numTrisPerStrip = (width / rez) * 2 - 2;

	// first, configure the cube's VAO (and terrainVBO + terrainIBO)
	unsigned int terrainVAO, terrainVBO, terrainIBO;
	glGenVertexArrays(1, &terrainVAO);
	glBindVertexArray(terrainVAO);

	glGenBuffers(1, &terrainVBO);
	glBindBuffer(GL_ARRAY_BUFFER, terrainVBO);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), &vertices[0], GL_STATIC_DRAW);

	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	glGenBuffers(1, &terrainIBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, terrainIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned), &indices[0], GL_STATIC_DRAW);



	// Rendering loop
	const int rot_offset = 5;
	int acc_rot = 0;
	const int num_extra_frames = 5;
	int frame_counter = 0;
	int windowWidth, windowHeight;
	//main loop
	Renderer* renderer = (Renderer*)glfwGetWindowUserPointer(window);
	glfwGetWindowSize(window, &windowWidth, &windowHeight);
	renderer->post_resize(window, windowWidth, windowHeight);
	for (int i = 0; i < renderer->GetScene()->data_list.size(); i++)
		renderer->core().toggle(renderer->GetScene()->data_list[i].show_lines);

	while (!glfwWindowShouldClose(window))
	{
		double tic = igl::get_seconds();
		renderer->Animate();
		renderer->draw(window);

		// update explosion generator
		if (tic - last_explosion_time > 2) {
			delete(explosion);
			explosion = nullptr;
		}

		if (tic - last_explosion_time > 10)
			renderer->GetScene()->isFog = true;
		

		// first position rotation
		if (renderer->GetScene()->update_camera_rotation && renderer->GetScene()->isFP) {

			if (acc_rot == 120) {
				renderer->GetScene()->update_camera_rotation = false;
				acc_rot = 0;
			}

			acc_rot += rot_offset;

			renderer->GetScene()->keyPressed == 'r' ? camera.ProcessMouseMovement(rot_offset, 0)  :
			renderer->GetScene()->keyPressed == 'l' ? camera.ProcessMouseMovement(-rot_offset, 0) :
			renderer->GetScene()->keyPressed == 'u' ? camera.ProcessMouseMovement(0, rot_offset)  :
			renderer->GetScene()->keyPressed == 'd' ? camera.ProcessMouseMovement(0, -rot_offset) :
													  camera.ProcessMouseMovement(0, 0);
		}

		// per-frame time logic
		// --------------------
		float currentFrame = static_cast<float>(glfwGetTime());
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;


		// input
		// -----
		processInput(window);

		//glViewport(0, 0, VIEWPORT_WIDTH, VIEWPORT_HEIGHT);
		glm::mat4 view = glm::mat4(glm::mat3(camera.GetViewMatrix()));
		glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);

		// cubemap shader
		glDepthFunc(GL_LEQUAL);  // change depth function so depth test passes when values are equal to depth buffer's content
		skyboxShader.use();
		skyboxShader.setMat4("view", view);
		skyboxShader.setMat4("projection", projection);
		// skybox cube
		glBindVertexArray(skyboxVAO);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		glBindVertexArray(0);
		glDepthFunc(GL_LESS); // set depth function back to default

		// draw background
		//glViewport((VIEWPORT_WIDTH / 4) * 3, VIEWPORT_HEIGHT / 5, VIEWPORT_WIDTH / 4 * 1, VIEWPORT_HEIGHT / 5);




		// heightmap
		// ------
		//glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// be sure to activate shader when setting uniforms/drawing objects
		heightMapShader.use();

		// view/projection transformations
		heightMapShader.setMat4("projection", projection);
		heightMapShader.setMat4("view", view);

		// world transformation
		glm::mat4 model = glm::mat4(1.0f);
		heightMapShader.setMat4("model", model);

		// render the cube
		glBindVertexArray(terrainVAO);
		//         glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);   // enable for wireframes
		for (unsigned strip = 0; strip < numStrips; strip++)
		{
			glDrawElements(GL_TRIANGLE_STRIP,   // primitive type
				numTrisPerStrip + 2,   // number of indices to render
				GL_UNSIGNED_INT,     // index data type
				(void*)(sizeof(unsigned) * (numTrisPerStrip + 2) * strip)); // offset to starting index
		}




		 // fog shader
		if (renderer->GetScene()->isFog) {

			// capture camera y rotation
			// --------------------------
			float camera_y_angle = camera.Up[2];
			camera_y_angle < -0.3 ? particleGen->set_camera_angle(0) :
				camera_y_angle < -0.2 ? particleGen->set_camera_angle(1) :
				camera_y_angle < -0.1 ? particleGen->set_camera_angle(2) :
				camera_y_angle < 0 ? particleGen->set_camera_angle(3) :
				camera_y_angle < 0.1 ? particleGen->set_camera_angle(4) : particleGen->set_camera_angle(5);

			// update particle positions
			particleGen->Update(0.1f, 10);

			// draw particles	
			// use additive blending to give it a 'glow' effect
			glBlendFunc(GL_SRC_ALPHA, GL_ONE);
			fogShader.use();
			for (Particle particle : particleGen->particles)
			{
				if (particle.Life > 0.0f)
				{
					fogShader.setVec2("offset", particle.Position);
					fogShader.setVec4("color", particle.Color);
					fogTexture.Bind();
					glBindVertexArray(particleGen->VAO);
					glDrawArrays(GL_TRIANGLES, 0, 6);
					glBindVertexArray(0);
				}
			}
			// don't forget to reset to default blending mode
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		}

		if (explosion) {
			// update particle positions
			explosion->Update(0.1f, 10);

			// draw particles	
			// use additive blending to give it a 'glow' effect
			glBlendFunc(GL_SRC_ALPHA, GL_ONE);
			fogShader.use();
			for (Particle particle : explosion->particles)
			{
				if (particle.Life > 0.0f)
				{
					fogShader.setVec2("offset", particle.Position);
					fogShader.setVec4("color", particle.Color);
					fogTexture.Bind();
					glBindVertexArray(explosion->VAO);
					glDrawArrays(GL_TRIANGLES, 0, 6);
					glBindVertexArray(0);
				}
			}
			// don't forget to reset to default blending mode
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		}

		

		// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
		// -------------------------------------------------------------------------------
		glfwSwapBuffers(window);


		if (renderer->core().is_animating || frame_counter++ < num_extra_frames)
		{//motion
			glfwPollEvents();
			// In microseconds
			double duration = 1000000. * (igl::get_seconds() - tic);
			const double min_duration = 1000000. / renderer->core().animation_max_fps;
			if (duration < min_duration)
			{
				std::this_thread::sleep_for(std::chrono::microseconds((int)(min_duration - duration)));
			}
		}
		else
		{
			glfwPollEvents();
			frame_counter = 0;
		}
		if (!loop)
			return !glfwWindowShouldClose(window);

#ifdef __APPLE__
		static bool first_time_hack = true;
		if (first_time_hack) {
			glfwHideWindow(window);
			glfwShowWindow(window);
			first_time_hack = false;
		}
#endif
	}

	glDeleteVertexArrays(1, &skyboxVAO);
	glDeleteBuffers(1, &skyboxVBO);

	glDeleteVertexArrays(1, &terrainVAO);
	glDeleteBuffers(1, &terrainVBO);
	glDeleteBuffers(1, &terrainIBO);
	delete(particleGen); 

	if(explosion)
		delete(explosion);

	return EXIT_SUCCESS;
}

// process all input : query GLFW whether relevant keys are pressed / released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow* window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		camera.ProcessKeyboard(FORWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		camera.ProcessKeyboard(BACKWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		camera.ProcessKeyboard(LEFT, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		camera.ProcessKeyboard(RIGHT, deltaTime);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}


// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_move(GLFWwindow* window, double x, double y)
{
	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);

	float xpos = static_cast<float>(x);
	float ypos = static_cast<float>(y);
	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos; 

	lastX = xpos;
	lastY = ypos;

	camera.ProcessMouseMovement(xoffset, yoffset);

}


// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, int button, int action, int modifier)
{
	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	igl::opengl::glfw::Viewer* scn = rndr->GetScene();

	if (action == GLFW_PRESS)
	{
		double x2, y2;
		glfwGetCursorPos(window, &x2, &y2);

		double depth, closestZ = 1;
		int i = 0;

		for (; i < scn->data_list.size(); i++)
		{
			scn->selected_data_index = i;
			depth = rndr->Picking(x2, y2);

			if (depth < 0 && (closestZ > 0 || closestZ < depth))
			{
				scn->current_picked = i;
				closestZ = depth;

				if (scn->data_list[i].type != NONE && scn->data_list[i].type != BASIC) {

					scn->data_list[i].is_visible = false;
					last_explosion_time = igl::get_seconds();
					explosion = new ParticleGenerator(500, true, 0.676 * x2 - 311.6, 0.73 * y2 + 7);

					if (scn->data_list[i].type == BEZIER) 
						rndr->GetScene()->isFog = false;
					
					scn->add_score(scn->data_list[i].type);

					PlaySound(TEXT("C:/Users/pijon/OneDrive/Desktop/animation3D/tutorial/sounds/SHEESH.wav"), NULL, SND_NODEFAULT | SND_ASYNC);
				}
			}
		}

	}


}


// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	if (rndr->IsPicked())
		if (rndr->GetScene()->selected_data_index == 0)
			rndr->GetScene()->data().MyTranslateInSystem(rndr->GetScene()->GetRotation(), Eigen::Vector3d(0, 0, yoffset));
		else
			rndr->GetScene()->data_list[1].MyTranslateInSystem(rndr->GetScene()->GetRotation(), Eigen::Vector3d(0, 0, yoffset));
	else {
		rndr->GetScene()->MyTranslate(Eigen::Vector3d(0, 0, yoffset * 0.5), true);
		camera.ProcessMouseScroll(static_cast<float>(yoffset * 0.5));
	}

}


void Display::AddKeyCallBack(void(*keyCallback)(GLFWwindow*, int, int, int, int))
{
	glfwSetKeyCallback(window, (void(*)(GLFWwindow*, int, int, int, int))keyCallback);

}

void Display::AddMouseCallBacks(void (*mousebuttonfun)(GLFWwindow*, int, int, int), void (*scrollfun)(GLFWwindow*, double, double), void (*cursorposfun)(GLFWwindow*, double, double))
{
	glfwSetMouseButtonCallback(window, mousebuttonfun);
	glfwSetScrollCallback(window, scrollfun);
	glfwSetCursorPosCallback(window, cursorposfun);
}

void Display::AddResizeCallBack(void (*windowsizefun)(GLFWwindow*, int, int))
{
	glfwSetWindowSizeCallback(window, windowsizefun);
}

void Display::SetRenderer(void* userPointer)
{
	renderer = userPointer;
	glfwSetWindowUserPointer(window, userPointer);

}

void* Display::GetScene()
{
	return glfwGetWindowUserPointer(window);
}

void Display::SwapBuffers()
{
	glfwSwapBuffers(window);
}

void Display::PollEvents()
{
	glfwPollEvents();
}

Display::~Display()
{
	glfwDestroyWindow(window);
	glfwTerminate();
}

// utility function for loading a 2D texture from file
// ---------------------------------------------------
Texture2D loadTexture(char const* path, bool alpha)
{
	// create texture object
	Texture2D texture;
	if (alpha)
	{
		texture.Internal_Format = GL_RGBA;
		texture.Image_Format = GL_RGBA;
	}
	// load image
	int width, height, nrChannels;
	unsigned char* data = stbi_load(path, &width, &height, &nrChannels, 0);

	if (data) {
		// now generate texture
		texture.Generate(width, height, data);
	}
	else {
		std::cout << "Texture failed to load at path: " << path << std::endl;
	}

	// and finally free image data
	stbi_image_free(data);

	return texture;
}

// loads a cubemap texture from 6 individual texture faces
// order:
// +X (right)
// -X (left)
// +Y (top)
// -Y (bottom)
// +Z (front) 
// -Z (back)
// -------------------------------------------------------
unsigned int loadCubemap(std::vector<std::string> faces)
{
	unsigned int textureID;
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

	int width, height, nrChannels;
	for (unsigned int i = 0; i < faces.size(); i++)
	{
		unsigned char* data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
		if (data)
		{
			glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
			stbi_image_free(data);
		}
		else
		{
			std::cout << "Cubemap texture failed to load at path: " << faces[i] << std::endl;
			stbi_image_free(data);
		}
	}
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	return textureID;
}
