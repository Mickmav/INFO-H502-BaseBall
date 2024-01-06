#include<iostream>
// TODO 
// Framebuffer
// Shadows
// Optional : geometry shader
//include glad before GLFW to avoid header conflict or define "#define GLFW_INCLUDE_NONE"
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "camera.h"
#include "shader.h"
#include "object.h"


#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include <map>


int width = 1500;
int height = 1000;
bool giveImpulseB = false;
bool giveImpulseN = false;
bool giveImpulseV = false;
bool ballExceededThreshold = false;
int initialLifetime = 10;
float deltaTime2 = 0;
bool transitionBall = false;
bool previousStateV = false;
btAlignedObjectArray<btCollisionShape*> collisionShapes;

/*
// Declare framebuffer ID and texture ID for secondary view
GLuint secondaryFramebuffer;
GLuint secondaryColorTexture;

// Initialize the secondary framebuffer
void initSecondaryFramebuffer() {
	glGenFramebuffers(1, &secondaryFramebuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, secondaryFramebuffer);

	// Create a color texture for the secondary view
	glGenTextures(1, &secondaryColorTexture);
	glBindTexture(GL_TEXTURE_2D, secondaryColorTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width / 2, height / 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Attach the color texture to the framebuffer
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, secondaryColorTexture, 0);

	// Check framebuffer completeness
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		std::cerr << "Secondary Framebuffer is not complete!" << std::endl;
	}

	// Unbind framebuffer
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}



void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
	// Update the viewport for the main framebuffer
	glViewport(0, 0, width, height);

	// Update the viewport for the secondary framebuffer (bottom right corner)
	glViewport(width / 2, 0, width / 2, height / 2);
}


GLuint framebuffer;

void initFramebuffer() {
	// Generate framebuffer
	glGenFramebuffers(1, &framebuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

	// Create a color attachment texture
	GLuint textureColorBuffer;
	glGenTextures(1, &textureColorBuffer);
	glBindTexture(GL_TEXTURE_2D, textureColorBuffer);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, screenWidth, screenHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureColorBuffer, 0);

	// Create a renderbuffer object for depth and stencil attachment
	GLuint rbo;
	glGenRenderbuffers(1, &rbo);
	glBindRenderbuffer(GL_RENDERBUFFER, rbo);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, screenWidth, screenHeight);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);

	// Check if framebuffer is complete
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		std::cerr << "Framebuffer is not complete!" << std::endl;
	}

	// Unbind framebuffer
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

*/


struct Particle {
	glm::vec3 position;
	glm::vec3 velocity;  // Initial velocity
	float lifetime;  // Time until the particle disappears
};


GLuint compileShader(std::string shaderCode, GLenum shaderType);
GLuint compileProgram(GLuint vertexShader, GLuint fragmentShader);

void processInput(GLFWwindow* window);
void loadCubemapFace(const char* file, const GLenum& targetCube);
btRigidBody* createSphere(btDynamicsWorld* dynamicsWorld, btScalar mass, const btVector3& position);



#ifndef NDEBUG
void APIENTRY glDebugOutput(GLenum source,
	GLenum type,
	unsigned int id,
	GLenum severity,
	GLsizei length,
	const char* message,
	const void* userParam)
{
	// ignore non-significant error/warning codes
	if (id == 131169 || id == 131185 || id == 131218 || id == 131204) return;

	std::cout << "---------------" << std::endl;
	std::cout << "Debug message (" << id << "): " << message << std::endl;

	switch (source)
	{
	case GL_DEBUG_SOURCE_API:             std::cout << "Source: API"; break;
	case GL_DEBUG_SOURCE_WINDOW_SYSTEM:   std::cout << "Source: Window System"; break;
	case GL_DEBUG_SOURCE_SHADER_COMPILER: std::cout << "Source: Shader Compiler"; break;
	case GL_DEBUG_SOURCE_THIRD_PARTY:     std::cout << "Source: Third Party"; break;
	case GL_DEBUG_SOURCE_APPLICATION:     std::cout << "Source: Application"; break;
	case GL_DEBUG_SOURCE_OTHER:           std::cout << "Source: Other"; break;
	} std::cout << std::endl;

	switch (type)
	{
	case GL_DEBUG_TYPE_ERROR:               std::cout << "Type: Error"; break;
	case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR: std::cout << "Type: Deprecated Behaviour"; break;
	case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:  std::cout << "Type: Undefined Behaviour"; break;
	case GL_DEBUG_TYPE_PORTABILITY:         std::cout << "Type: Portability"; break;
	case GL_DEBUG_TYPE_PERFORMANCE:         std::cout << "Type: Performance"; break;
	case GL_DEBUG_TYPE_MARKER:              std::cout << "Type: Marker"; break;
	case GL_DEBUG_TYPE_PUSH_GROUP:          std::cout << "Type: Push Group"; break;
	case GL_DEBUG_TYPE_POP_GROUP:           std::cout << "Type: Pop Group"; break;
	case GL_DEBUG_TYPE_OTHER:               std::cout << "Type: Other"; break;
	} std::cout << std::endl;

	switch (severity)
	{
	case GL_DEBUG_SEVERITY_HIGH:         std::cout << "Severity: high"; break;
	case GL_DEBUG_SEVERITY_MEDIUM:       std::cout << "Severity: medium"; break;
	case GL_DEBUG_SEVERITY_LOW:          std::cout << "Severity: low"; break;
	case GL_DEBUG_SEVERITY_NOTIFICATION: std::cout << "Severity: notification"; break;
	} std::cout << std::endl;
	std::cout << std::endl;
}
#endif

Camera camera(glm::vec3(0.0, 5.0, 15));


int main(int argc, char* argv[])
{
	std::cout << "Welcome to our projet baseball demo " << std::endl;
	std::cout << " This code is an example of the bullet library \n";

	//Create the OpenGL context 
	if (!glfwInit()) {
		throw std::runtime_error("Failed to initialise GLFW \n");
	}
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifndef NDEBUG
	//create a debug context to help with Debugging
	glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, true);
#endif


	//Create the window
	GLFWwindow* window = glfwCreateWindow(width, height, "Solution 02", nullptr, nullptr);
	if (window == NULL)
	{
		glfwTerminate();
		throw std::runtime_error("Failed to create GLFW window\n");
	}

	glfwMakeContextCurrent(window);

	//load openGL function
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		throw std::runtime_error("Failed to initialize GLAD");
	}

	glEnable(GL_DEPTH_TEST);

#ifndef NDEBUG
	int flags;
	glGetIntegerv(GL_CONTEXT_FLAGS, &flags);
	if (flags & GL_CONTEXT_FLAG_DEBUG_BIT)
	{
		glEnable(GL_DEBUG_OUTPUT);
		glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
		glDebugMessageCallback(glDebugOutput, nullptr);
		glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);
	}
#endif

	const std::string sourceV = "#version 330 core\n"
		"in vec3 position; \n"
		"in vec2 tex_coord; \n"
		"in vec3 normal; \n"
		"out vec4 v_col; \n"
		"out vec2 v_t; \n"
		"uniform mat4 M; \n"
		"uniform mat4 V; \n"
		"uniform mat4 P; \n"
		" void main(){ \n"
		"gl_Position = P*V*M*vec4(position, 1);\n"
		"v_col = vec4(normal*0.5 + 0.5, 1.0);\n"
		"v_t = tex_coord; \n"
		"}\n";

	const std::string sourceV2 = "#version 330 core\n"
		"in vec3 position; \n"
		"in vec2 tex_coords; \n"
		"in vec3 normal; \n"

		"out vec3 v_frag_coord; \n"
		"out vec3 v_normal; \n"

		"uniform mat4 M; \n"
		"uniform mat4 itM; \n"
		"uniform mat4 V; \n"
		"uniform mat4 P; \n"

		" void main(){ \n"
		"vec4 frag_coord = M*vec4(position, 1.0); \n"
		"gl_Position = P*V*frag_coord; \n"
		"v_normal = vec3(itM * vec4(normal, 1.0)); \n"
		"v_frag_coord = frag_coord.xyz; \n"
		"\n"
		"}\n";

	const std::string sourceF = "#version 330 core\n"
		"out vec4 FragColor;"
		"precision mediump float; \n"
		"in vec4 v_col; \n"
		"in vec2 v_t; \n"
		"void main() { \n"
		"FragColor = v_col*(1.0-v_t.y); \n"
		"} \n";

	const std::string sourceF2 = "#version 400 core\n"
		"out vec4 FragColor;\n"
		"precision mediump float; \n"

		"in vec3 v_frag_coord; \n"
		"in vec3 v_normal; \n"

		"uniform vec3 u_view_pos; \n"

		"uniform samplerCube cubemapSampler; \n"


		"void main() { \n"
		// For refraction "float ratio = 1.00 / refractionIndice;\n"
		"vec3 N = normalize(v_normal);\n"
		"vec3 V = normalize(u_view_pos - v_frag_coord); \n"
		"vec3 R = reflect(-V,N); \n"
		// For refraction "vec3 R = refract(-V,N,ratio); \n"
		"FragColor = texture(cubemapSampler,R); \n"
		"} \n";



	const std::string sourceVCubeMap = "#version 330 core\n"
		"in vec3 position; \n"
		"in vec2 tex_coords; \n"
		"in vec3 normal; \n"
		"uniform mat4 V; \n"
		"uniform mat4 P; \n"
		"out vec3 texCoord_v; \n"

		" void main(){ \n"
		"texCoord_v = position;\n"
		"mat4 V_no_rot = mat4(mat3(V)) ;\n" //remove translation info from view matrix to only keep rotation
		"vec4 pos = P * V_no_rot * vec4(position, 1.0); \n"
		// the positions xyz are divided by w after the vertex shader
		// the z component is equal to the depth value
		// we want a z always equal to 1.0 here, so we set z = w!
		// Remember: z=1.0 is the MAXIMUM depth value
		"gl_Position = pos.xyww;\n"
		"\n"
		"}\n";

	const std::string sourceFCubeMap =
		"#version 330 core\n"
		"out vec4 FragColor;\n"
		"precision mediump float; \n"
		"uniform samplerCube cubemapSampler; \n"
		"in vec3 texCoord_v; \n"
		"void main() { \n"
		"FragColor = texture(cubemapSampler,texCoord_v); \n"
		"} \n";


	Shader shader(sourceV, sourceF);
	Shader shader2(sourceV2, sourceF2);
	Shader cubeMapShader = Shader(sourceVCubeMap, sourceFCubeMap);

	char pathCube[] = PATH_TO_OBJECTS "/cube.obj";
	Object cubeMap(pathCube);
	cubeMap.makeObject(cubeMapShader);

	char pathSphere[] = PATH_TO_OBJECTS "/sphere_smooth.obj";
	Object sphere(pathSphere);
	sphere.makeObject(shader);

	char pathBatter[] = PATH_TO_OBJECTS "/basball.obj";
	Object cube2(pathBatter);
	cube2.makeObject(shader);

	char pathPlane[] = PATH_TO_OBJECTS "/plane.obj";
	Object plane(pathPlane);
	plane.makeObject(shader);




	double prev = 0;
	int deltaFrame = 0;
	//fps function
	auto fps = [&](double now) {
		deltaTime2 = now - prev;
		double deltaTime = now - prev;
		deltaFrame++;
		if (deltaTime > 0.5) {
			prev = now;
			const double fpsCount = (double)deltaFrame / deltaTime;
			deltaFrame = 0;
			std::cout << "\r FPS: " << fpsCount;
		}
	};



	glm::vec3 light_pos = glm::vec3(1.0, 2.0, 1.5);
	glm::mat4 model = glm::mat4(1.0);
	model = glm::translate(model, glm::vec3(0.0, 0.0, -2.0));
	model = glm::scale(model, glm::vec3(0.5, 0.5, 0.5));

	glm::mat4 inverseModel = glm::transpose(glm::inverse(model));

	glm::mat4 view = camera.GetViewMatrix();
	glm::mat4 perspective = camera.GetProjectionMatrix();

	float ambient = 0.1;
	float diffuse = 0.5;
	float specular = 0.8;

	glm::vec3 materialColour = glm::vec3(0.5f, 0.6, 0.8);
	// test autre couleur glm::vec3 materialColour = glm::vec3(0.f, 0., 0.);




	shader2.use();
	shader2.setFloat("shininess", 32.0f);
	shader2.setVector3f("materialColour", materialColour);
	shader2.setFloat("light.ambient_strength", ambient);
	shader2.setFloat("light.diffuse_strength", diffuse);
	shader2.setFloat("light.specular_strength", specular);
	shader2.setFloat("light.constant", 1.0);
	shader2.setFloat("light.linear", 0.14);
	shader2.setFloat("light.quadratic", 0.07);
	// For refraction shader2.setFloat("refractionIndice", 1.52);



	//1. Create bullet world 

	///-----initialization_start-----
	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();
	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	//Finally create the world
	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

	dynamicsWorld->setGravity(btVector3(0, -1, 0));


	//2. Create the collisions shapes
	glm::mat4 model2 = glm::mat4(2.0);
	glm::mat4 modelGround = glm::mat4(1.0);
	modelGround = glm::scale(modelGround, glm::vec3(10, 1, 10));
	modelGround = glm::translate(modelGround, glm::vec3(0, 0, 0));

	//2.a We need one for the ground
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(100.), btScalar(50.), btScalar(100.)));
		collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, -50, 0));
		btScalar mass(0.);
		bool isDynamic = false;
		btVector3 localInertia(0, 0, 0);

		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
		rbInfo.m_restitution = 1.0f;
		btRigidBody* body = new btRigidBody(rbInfo);

		dynamicsWorld->addRigidBody(body);
	}
	//2.b 3 other ones are for the 3 balls of the game
	btRigidBody* bodySphere = createSphere(dynamicsWorld, 1.0, btVector3(-4, 10, -3.5));

	
	//3.b Another one for the batter
	btRigidBody* bodyBatter;
	{
		btCollisionShape* colShape = new btBoxShape(btVector3(btScalar(5.), btScalar(0.1), btScalar(3.)));
		collisionShapes.push_back(colShape);
		btScalar mass(50000);
		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(btVector3(0, 3, 0));// 10 1 10
		startTransform.setRotation(btQuaternion(0, 0, 0, mass));
		bool isDynamic = (mass != 0.f);
		btVector3 localInertia(100, 0, 100);
		// objects of infinite mass can't move or rotate
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);
		//btDefaultMotionState* myMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, mass), btVector3(0, 3, 0)));

		// create the motion state from the initial transform
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

		// create the rigid body construction info using the mass, motion state and shape
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
		rbInfo.m_restitution = 1.0f;

		// create the rigid body
		bodyBatter = new btRigidBody(rbInfo);

		dynamicsWorld->addRigidBody(bodyBatter);
	}

	glfwSwapInterval(1);
	//Rendering

	GLuint cubeMapTexture;
	glGenTextures(1, &cubeMapTexture);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_CUBE_MAP, cubeMapTexture);

	// texture parameters
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	//stbi_set_flip_vertically_on_load(true);

	std::string pathToCubeMap = PATH_TO_TEXTURE "/cubemaps/yokohama3/";

	std::map<std::string, GLenum> facesToLoad = {
		{pathToCubeMap + "posx.jpg",GL_TEXTURE_CUBE_MAP_POSITIVE_X},
		{pathToCubeMap + "posy.jpg",GL_TEXTURE_CUBE_MAP_POSITIVE_Y},
		{pathToCubeMap + "posz.jpg",GL_TEXTURE_CUBE_MAP_POSITIVE_Z},
		{pathToCubeMap + "negx.jpg",GL_TEXTURE_CUBE_MAP_NEGATIVE_X},
		{pathToCubeMap + "negy.jpg",GL_TEXTURE_CUBE_MAP_NEGATIVE_Y},
		{pathToCubeMap + "negz.jpg",GL_TEXTURE_CUBE_MAP_NEGATIVE_Z},
	};
	//load the six faces
	for (std::pair<std::string, GLenum> pair : facesToLoad) {
		loadCubemapFace(pair.first.c_str(), pair.second);
	}

	auto lastFrameTime = glfwGetTime();

	std::vector<Particle> ballTrailParticles;
	btVector3 initialVelocity = bodySphere->getLinearVelocity();
	btVector3 previousVelocity = initialVelocity;
	float speedChangeThreshold = 2.0f;



	// Particle object
	// 1. Triangle vertices
	const float positionsData[9] = {
		// vertices
		-1.0, -1.0, 0.0,
		1.0, -1.0, 0.0,
		0.0, 1.0, 0.0,
	};

	//Create the buffer
	// 2. Your code: make a VBO (buffer) and a VAO (vertex buffer object) and send the data from the positionData to the GPU
	GLuint VBO, VAO;
	//generate the buffer and the vertex array
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);

	//define VBO and VAO as active buffer and active vertex array
	glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(positionsData), positionsData, GL_STATIC_DRAW);

	//Specify the vertex attributes
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, false, 0, 0);

	//desactive the buffer
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	


	/*
	// Initialization
	initFramebuffer();
	initSecondaryFramebuffer();  // Call this once during initialization

	// Set the callback function for window resize
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	*/


	while (!glfwWindowShouldClose(window)) {

		processInput(window);

		glfwPollEvents();

		double now = glfwGetTime();


		glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		glBindVertexArray(VAO);


		//3. Ask bullet to do the simulation
		dynamicsWorld->stepSimulation(now - lastFrameTime, 10);

		// Ball change
		if (giveImpulseV && (previousStateV == false)) {
			transitionBall = true;
			previousStateV = true;
		}
		else if (!giveImpulseV) {
			previousStateV = false;
		}


		if (transitionBall) {
			// Move the ball to a specific position
			btTransform newTransform;
			newTransform.setIdentity();
			newTransform.setOrigin(btVector3(-4, 10, -3.5));
			bodySphere->getMotionState()->setWorldTransform(newTransform);
			bodySphere->setWorldTransform(newTransform);
			bodySphere->setLinearVelocity(btVector3(0, 0, 0));
			bodySphere->setAngularVelocity(btVector3(0, 0, 0));

			btVector3 previousVelocity = initialVelocity;
			ballTrailParticles.clear();
			ballExceededThreshold = false;
			transitionBall = false;
		}

		// Compare the actual speed with the previous one
		btVector3 currentVelocity = bodySphere->getLinearVelocity();
		float speedChange = (currentVelocity - previousVelocity).length();

		// Check if speed is above the threshold
		if (speedChange > speedChangeThreshold) {
			ballExceededThreshold = true;
		}
		else {
			previousVelocity = currentVelocity;   // Update the speed
		}

		//4. Get the model matrice for the object (batter) from bullet

		btTransform transform2;
		bodyBatter->getMotionState()->getWorldTransform(transform2);
		transform2.getOpenGLMatrix(glm::value_ptr(model2));

		bodyBatter->setLinearVelocity(btVector3(0, 0, 0));

		// Batter movements
		if (giveImpulseB || giveImpulseN) {
			if (giveImpulseB) {
				bodyBatter->setAngularVelocity(btVector3(0, -15, 0));
				giveImpulseB = false;
			}
			else {
				bodyBatter->setAngularVelocity(btVector3(0, 5, 0));
				giveImpulseN = false;
			}
		}
		else {
			bodyBatter->setAngularVelocity(btVector3(0, 0, 0));
		}



		glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


		view = camera.GetViewMatrix();

		// Use the shader Class to send the uniform
		shader2.use();
		btTransform transform;
		bodySphere->getMotionState()->getWorldTransform(transform);
		transform.getOpenGLMatrix(glm::value_ptr(model));
		shader2.setMatrix4("M", model);
		shader2.setMatrix4("itM", inverseModel);
		shader2.setMatrix4("V", view);
		shader2.setMatrix4("P", perspective);
		shader2.setVector3f("u_view_pos", camera.Position);

		auto delta = light_pos + glm::vec3(0.0, 0.0, 2 * std::sin(now));


		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, cubeMapTexture);
		cubeMapShader.setInteger("cubemapTexture", 0);

		glm::vec3 ballPosition2(
			transform.getOrigin().getX(),
			transform.getOrigin().getY(),
			transform.getOrigin().getZ()
		);
		sphere.draw();



		shader.use();
		shader.setMatrix4("V", view);
		shader.setMatrix4("P", perspective);

		if (ballExceededThreshold) {
			btTransform transform;
			bodySphere->getMotionState()->getWorldTransform(transform);
			glm::vec3 ballPosition(
				transform.getOrigin().getX(),
				transform.getOrigin().getY(),
				transform.getOrigin().getZ()
			);

			// Add a new particle to the trail
			Particle newParticle;
			newParticle.position = (ballPosition);
			newParticle.velocity = glm::vec3(0.01f, -0.01f, 0.01f);
			newParticle.lifetime = initialLifetime;
			ballTrailParticles.push_back(newParticle);
		};



		for (auto& particle : ballTrailParticles) {

			particle.lifetime -= deltaTime2;
			particle.position += particle.velocity * deltaTime2; // Update position
			particle.velocity.y -= 9.81 * deltaTime2 / 1000; // Apply gravity

			glm::mat4 model = glm::translate(glm::mat4(1.0f), particle.position);
			shader.setMatrix4("M", model);
			glDrawArrays(GL_TRIANGLES, 0, 30);

			if (particle.lifetime <= 0.0f) {
				ballTrailParticles.erase(ballTrailParticles.begin(), ballTrailParticles.begin() + 1);
			}

		}


		// draw batter
		shader.setMatrix4("M", model2);
		cube2.draw();

		// draw ground
		shader.setMatrix4("M", modelGround);
		plane.draw();

		// draw cubemap
		cubeMapShader.use();
		cubeMapShader.setMatrix4("V", view);
		cubeMapShader.setMatrix4("P", perspective);
		cubeMapShader.setInteger("cubemapTexture", 0);

		glDepthFunc(GL_LEQUAL);  // Set depth function to less than or equal for cubemap
		cubeMap.draw();
		glDepthFunc(GL_LESS);    // Reset depth function to default
		fps(now);
		lastFrameTime = now;
		glfwSwapBuffers(window);
	}
		/*
		// Render to the secondary framebuffer
		glBindFramebuffer(GL_FRAMEBUFFER, secondaryFramebuffer);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glViewport(width / 2, 0, width / 2, height / 2);

		// Set up the secondary camera (same position or orientation as the main camera)
		view = camera.GetViewMatrix();  // Use the same camera

		// Enable a shader for the black and white effect
		shader.use();

		// Pass necessary uniforms to the shader, if required

		// Render the scene 

		// Switch back to the main framebuffer
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glViewport(0, 0, width, height);
		*/


	//5. Clean up 
	//remove the rigidbodies from the dynamics world and delete them
	for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())	{
			delete body->getMotionState();
		}
		dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}
	//delete collision shapes
	for (int j = 0; j < collisionShapes.size(); j++)
	{
		btCollisionShape* shape = collisionShapes[j];
		collisionShapes[j] = 0;
		delete shape;
	}
	//delete dynamics world
	delete dynamicsWorld;
	//delete solver
	delete solver;
	//delete broadphase
	delete overlappingPairCache;
	//delete dispatcher
	delete dispatcher;
	delete collisionConfiguration;
	//next line is optional: it will be cleared by the destructor when the array goes out of scope
	collisionShapes.clear();


	//clean up ressource
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}


void processInput(GLFWwindow* window) {
	// Use the cameras class to change the parameters of the camera
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		camera.ProcessKeyboardMovement(LEFT, 0.1);
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		camera.ProcessKeyboardMovement(RIGHT, 0.1);

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		camera.ProcessKeyboardMovement(FORWARD, 0.1);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		camera.ProcessKeyboardMovement(BACKWARD, 0.1);

	if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
		camera.ProcessKeyboardRotation(1, 0.0, 1);
	if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
		camera.ProcessKeyboardRotation(-1, 0.0, 1);

	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
		camera.ProcessKeyboardRotation(0.0, 1.0, 1);
	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
		camera.ProcessKeyboardRotation(0.0, -1.0, 1);


	if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS)
		giveImpulseN = true;
	if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS)
		giveImpulseB = true;
	if (glfwGetKey(window, GLFW_KEY_N) != GLFW_PRESS)
		giveImpulseN = false;
	if (glfwGetKey(window, GLFW_KEY_B) != GLFW_PRESS)
		giveImpulseB = false;

	if (glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS)
		giveImpulseV = true;
	if (glfwGetKey(window, GLFW_KEY_V) != GLFW_PRESS)
		giveImpulseV = false;


}




void loadCubemapFace(const char* path, const GLenum& targetFace)
{
	int imWidth, imHeight, imNrChannels;
	unsigned char* data = stbi_load(path, &imWidth, &imHeight, &imNrChannels, 0);
	if (data)
	{

		glTexImage2D(targetFace, 0, GL_RGB, imWidth, imHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
		//glGenerateMipmap(targetFace);
	}
	else {
		std::cout << "Failed to Load texture" << std::endl;
		const char* reason = stbi_failure_reason();
		std::cout << reason << std::endl;
	}
	stbi_image_free(data);
}

btRigidBody* createSphere(btDynamicsWorld* dynamicsWorld, btScalar mass, const btVector3& position) {
	btCollisionShape* colShape = new btSphereShape(btScalar(1.));
	collisionShapes.push_back(colShape);

	btTransform startTransform;
	startTransform.setIdentity();

	// Set the position
	startTransform.setOrigin(position);

	btVector3 localInertia(1, 1, 1);
	if (mass != 0.f)
		colShape->calculateLocalInertia(mass, localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
	rbInfo.m_friction = 1.f;
	rbInfo.m_restitution = 0.9f;

	btRigidBody* body = new btRigidBody(rbInfo);
	dynamicsWorld->addRigidBody(body);

	return body;
}
