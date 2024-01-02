#include<iostream>

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
#include "ball.h"


#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include <map>


const int width = 1000;
const int height = 1000;
bool giveImpulseB = false;
bool giveImpulseN = false;
bool ballExceededThreshold = false;
int initialLifetime = 1000;

struct Particle {
	glm::vec3 position;
	glm::vec3 velocity;  // Initial velocity
	glm::vec3 color;
	float lifetime;  // Time until the particle disappears
};


GLuint compileShader(std::string shaderCode, GLenum shaderType);
GLuint compileProgram(GLuint vertexShader, GLuint fragmentShader);

void processInput(GLFWwindow* window);
void loadCubemapFace(const char* file, const GLenum& targetCube);


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

Camera camera(glm::vec3(0.0, 1.0, 10));


int main(int argc, char* argv[])
{
	// asteroids
	/*unsigned int amount = 10;
	glm::mat4* modelMatrices;
	modelMatrices = new glm::mat4[amount];
	srand(static_cast<unsigned int>(glfwGetTime())); // initialize random seed
	float radius = 5.0;
	float offset = 2.5f;
	for (unsigned int i = 0; i < amount; i++)
	{
		glm::mat4 model = glm::mat4(1.0f);
		// 1. translation: displace along circle with 'radius' in range [-offset, offset]
		float angle = (float)i / (float)amount * 360.0f;
		float displacement = (rand() % (int)(2 * offset * 10)) / 10.0f - offset;
		float x = sin(angle) * radius + displacement;
		displacement = (rand() % (int)(2 * offset * 10)) / 10.0f - offset;
		float y = displacement * 0.9f; // keep height of asteroid field smaller compared to width of x and z
		displacement = (rand() % (int)(2 * offset * 10)) / 10.0f - offset;
		float z = cos(angle) * radius + displacement;
		model = glm::translate(model, glm::vec3(x, y, z));

		// 2. scale: Scale between 0.05 and 0.25f
		//float scale = static_cast<float>((rand() % 20) / 100.0 + 0.05);
		//model = glm::scale(model, glm::vec3(scale));

		// 3. rotation: add random rotation around a (semi)randomly picked rotation axis vector
		//float rotAngle = static_cast<float>((rand() % 360));
		//model = glm::rotate(model, rotAngle, glm::vec3(0.4f, 0.6f, 0.8f));

		// 4. now add to list of matrices
		modelMatrices[i] = model;
	}
	*/


	std::cout << "Welcome to the bullet demo " << std::endl;
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

	const std::string sourceF = "#version 330 core\n"
		"out vec4 FragColor;"
		"precision mediump float; \n"
		"in vec4 v_col; \n"
		"in vec2 v_t; \n"
		"void main() { \n"
		"FragColor = v_col*(1.0-v_t.y); \n"
		"} \n";


	/*	const std::string particleVertexShaderSource = "#version 330 core\n"
			"in vec3 position; \n"
			"in vec3 color; \n"  // Add color attribute
			"out vec4 v_col; \n"
			"uniform mat4 M; \n"
			"uniform mat4 V; \n"
			"uniform mat4 P; \n"
			"void main(){ \n"
			"    gl_Position = P * V * M * vec4(position, 1);\n"
			"    v_col = vec4(color, 1.0); \n"
			"} \n";

		const std::string particleFragmentShaderSource = "#version 330 core\n"
			"out vec4 FragColor;"
			"in vec4 v_col; \n"
			"void main() { \n"
			"    FragColor = v_col; \n"
			"} \n";
	*/


	const std::string sourceVCubeMap = "#version 330 core\n"
		"in vec3 position; \n"
		"in vec2 tex_coords; \n"
		"in vec3 normal; \n"
		"uniform mat4 V; \n"
		"uniform mat4 P; \n"
		"out vec3 texCoord_v; \n"

		" void main(){ \n"
		"texCoord_v = position;\n"
		//remove translation info from view matrix to only keep rotation
		"mat4 V_no_rot = mat4(mat3(V)) ;\n"
		"vec4 pos = P * V_no_rot * vec4(position, 1.0); \n"
		// the positions xyz are divided by w after the vertex shader
		// the z component is equal to the depth value
		// we want a z always equal to 1.0 here, so we set z = w!
		// Remember: z=1.0 is the MAXIMUM depth value ;)
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
	/*	Shader shaderParticle(particleVertexShaderSource, particleFragmentShaderSource);
	*/
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
		double deltaTime = now - prev;
		deltaFrame++;
		if (deltaTime > 0.5) {
			prev = now;
			const double fpsCount = (double)deltaFrame / deltaTime;
			deltaFrame = 0;
			std::cout << "\r FPS: " << fpsCount;
		}
		};


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
	glm::mat4 model = glm::mat4(1.0);
	glm::mat4 model2 = glm::mat4(2.0);
	glm::mat4 modelGround = glm::mat4(1.0);
	modelGround = glm::scale(modelGround, glm::vec3(10, 1, 10));
	modelGround = glm::translate(modelGround, glm::vec3(0, 0, 0));

	btAlignedObjectArray<btCollisionShape*> collisionShapes;
	//2.a We need one for the ground
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
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
	//2.b Another one is for the sphere
	btRigidBody* bodySphere;
	{
		btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		collisionShapes.push_back(colShape);
		btTransform startTransform;
		startTransform.setIdentity();
		btScalar mass(1);
		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);
		btVector3 localInertia(1, 1, 1);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);
		startTransform.setOrigin(btVector3(-4, 4.75, -3.5)); // -4, 13, 3.5 // -14 13 -7.5
		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
		rbInfo.m_friction = 1.f;
		rbInfo.m_restitution = 1.0f;
		bodySphere = new btRigidBody(rbInfo);
		dynamicsWorld->addRigidBody(bodySphere);
	}
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


	glm::mat4 view = camera.GetViewMatrix();
	glm::mat4 perspective = camera.GetProjectionMatrix();


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
	btVector3 previousVelocity = bodySphere->getLinearVelocity();
	float speedChangeThreshold = 2.0f;


	while (!glfwWindowShouldClose(window)) {
		processInput(window);

		glfwPollEvents();

		double now = glfwGetTime();

		//3. Ask bullet to do the simulation
		dynamicsWorld->stepSimulation(now - lastFrameTime, 10);


		// Compare the actual speed with the previous one
		btVector3 currentVelocity = bodySphere->getLinearVelocity();
		float speedChange = (currentVelocity - previousVelocity).length();

		// Check if speed is above the threshold
		if (speedChange > speedChangeThreshold) {
			ballExceededThreshold = true;
		}
		else {
			// Update the speed
			previousVelocity = currentVelocity;
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
			}
			else {
				bodyBatter->setAngularVelocity(btVector3(0, 5, 0));
			}
		}
		else {
			bodyBatter->setAngularVelocity(btVector3(0, 0, 0));
		}


		glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		view = camera.GetViewMatrix();
		// Use the shader Class to send the uniform
		shader.use();
		shader.setMatrix4("V", view);
		shader.setMatrix4("P", perspective);


		// Get the model matrice for the object (sphere) from bullet
		btTransform transform;
		bodySphere->getMotionState()->getWorldTransform(transform);
		transform.getOpenGLMatrix(glm::value_ptr(model));

		shader.setMatrix4("M", model);
		sphere.draw();

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
			newParticle.position = ballPosition;
			newParticle.color = glm::vec3(0.0f, 0.0f, 1.0f);
			newParticle.velocity = glm::vec3(
				// Random x velocity,
				// Random y velocity,
				// Random z velocity
				//10.0f, 10.0f, 10.0f
				1.0f, 1.0f, 1.0f
			);
			newParticle.lifetime = initialLifetime;
			ballTrailParticles.push_back(newParticle);
		};


		///*
		//// draw meteorites
		//for (unsigned int i = 0; i < amount; i++)
		//{
		//	shader.setMatrix4("M", modelMatrices[i]);
		//	sphere.draw();
		//}
		//*/


/*		shaderParticle.use();
		shaderParticle.setMatrix4("V", view);
		shaderParticle.setMatrix4("P", perspective);
*/
		for (auto& particle : ballTrailParticles) {

			float deltaTime = 10;
			particle.lifetime -= deltaTime;

			// Update position --- Cannot see the result ---
			//particle.position += particle.velocity * deltaTime;

			// Apply gravity
			particle.velocity.y -= 9.81 * deltaTime;

			// Change color ------ NOT WORKING ------
			float colorChangeFactor = particle.lifetime / initialLifetime;  // Value between 0 and 1
			particle.color *= colorChangeFactor;  // Diminish the color over time


			glm::mat4 model = glm::translate(glm::mat4(1.0f), particle.position);
			shader.setMatrix4("M", model);
			//sphere.draw();
			glDrawArrays(GL_POINTS, 0, 100);


			if (particle.lifetime <= 0.0f) {
				ballTrailParticles.erase(ballTrailParticles.begin(), ballTrailParticles.begin() + 1);
			}

		}

		//glDrawArrays(GL_POINTS, 0, ballTrailParticles.size());


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

	//5. Clean up 
	//remove the rigidbodies from the dynamics world and delete them
	for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
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

