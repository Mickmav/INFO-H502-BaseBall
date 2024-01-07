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


#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include <map>


int width = 1500;
int height = 1000;
float deltaTime2 = 0;
int initialLifetime = 10;
bool giveImpulseB = false;
bool giveImpulseN = false;
bool giveImpulseV = false;
bool ballExceededThreshold = false;
bool transitionBall = false;
bool previousStateV = false;
float speedChangeThreshold = 2.0f;
btVector3 initialVelocity, previousVelocity;


Camera camera(glm::vec3(0.0, 5.0, 15));
Camera camera2(glm::vec3(0.0, 5.0, -30), glm::vec3(0,1,0), 90);


struct Particle {
	glm::vec3 position;
	glm::vec3 velocity;  // Initial velocity
	float lifetime;  // Time until the particle disappears
};

std::vector<Particle> ballTrailParticles;
btAlignedObjectArray<btCollisionShape*> collisionShapes;

btRigidBody* createSphere(btDynamicsWorld* dynamicsWorld, btScalar mass, const btVector3& position);
btRigidBody* createBatter(btDynamicsWorld* dynamicsWorld);
btRigidBody* createPlaneRigidBody(btDynamicsWorld* dynamicsWorld, btScalar mass, const btVector3& position, const btVector3& size);

unsigned int loadTexture(const char* path);

void processInput(GLFWwindow* window);

void handleBallTransition(btRigidBody* bodySphere);
void updateBodyBatter(btRigidBody* bodyBatter);
void handleThresholdForParticles(btRigidBody* bodySphere);

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



int main(int argc, char* argv[])
{
	std::cout << "Welcome to our project baseball demo " << std::endl;
	std::cout << " This code use a example of the bullet library, and the learn opengl tutorial \n";
	std::cout << "Have a nice game !" << std::endl;

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
	GLFWwindow* window = glfwCreateWindow(width, height, "Baseball Demo", nullptr, nullptr);
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

	const std::string sourceV3 = "#version 330 core\n"
		"in vec2 position;  \n"
		"in vec2 tex_coord;  \n"
		"out vec2 TexCoords;  \n"
		"void main()  \n"
		"{  \n"
		"TexCoords = tex_coord;  \n"
		"gl_Position = vec4(position.x, position.y, 0.0, 1.0);  \n"
		"}  \n";

	const std::string sourceV2 = "#version 330 core\n"
		"in vec3 position; \n"
		"in vec2 tex_coord; \n"
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

	const std::string sourceF3 = "#version 330 core\n"
		"out vec4 FragColor; \n"
		"in vec2 TexCoords; \n"
		"uniform sampler2D screenTexture; \n"
		"void main() \n"
		"{ \n"
		"vec3 col = texture(screenTexture, TexCoords).rgb; \n"
		"FragColor = vec4(col, 1.0); \n"
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
		"in vec2 tex_coord; \n"
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

	const std::string sourceVShadowMap = "#version 330 core \n"
		"layout (location = 0) in vec3 position;\n"
		"layout (location = 1) in vec3 normal;\n"
		"layout (location = 2) in vec2 tex_coord;\n"
		"\n"
		"out vec2 TexCoords;\n"
		"\n"
		"out VS_OUT {\n"
		"    vec3 FragPos;\n"
		"    vec3 Normal;\n"
		"    vec2 TexCoords;\n"
		"    vec4 FragPosLightSpace;\n"
		"} vs_out;\n"
		"\n"
		"uniform mat4 projection;\n"
		"uniform mat4 view;\n"
		"uniform mat4 model;\n"
		"uniform mat4 lightSpaceMatrix;\n"
		"\n"
		"void main()\n"
		"{\n"
		"    vs_out.FragPos =  vec3(model * vec4(position, 1.0));\n"
		"    vs_out.Normal = transpose(inverse(mat3(model))) * normal;\n"
		"    vs_out.TexCoords = tex_coord;\n"
		"    vs_out.FragPosLightSpace = lightSpaceMatrix * vec4(vs_out.FragPos, 1.0);\n"
		"    gl_Position = projection * view * model * vec4(position, 1.0);\n"
		"}\n";

	const std::string sourceFShadowMap = "#version 330 core\n"
		"out vec4 FragColor;\n"
		"\n"
		"in VS_OUT {\n"
		"    vec3 FragPos;\n"
		"    vec3 Normal;\n"
		"    vec2 TexCoords;\n"
		"    vec4 FragPosLightSpace;\n"
		"} fs_in;\n"
		"\n"
		"uniform sampler2D diffuseTexture;\n"
		"uniform sampler2D shadowMap;\n"
		"\n"
		"uniform vec3 lightPos;\n"
		"uniform vec3 viewPos;\n"
		"\n"
		"float ShadowCalculation(vec4 fragPosLightSpace)\n"
		"{\n"
		"    // perform perspective divide\n"
		"    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;\n"
		"    // transform to [0,1] range\n"
		"    projCoords = projCoords * 0.5 + 0.5;\n"
		"    // get closest depth value from light's perspective (using [0,1] range fragPosLight as coords)\n"
		"    float closestDepth = texture(shadowMap, projCoords.xy).r; \n"
		"    // get depth of current fragment from light's perspective\n"
		"    float currentDepth = projCoords.z;\n"
		"    // calculate bias (based on depth map resolution and slope)\n"
		"    vec3 normals = normalize(fs_in.Normal);\n"
		"    vec3 lightDir = normalize(lightPos - fs_in.FragPos);\n"
		"    float bias = max(0.05 * (1.0 - dot(normals, lightDir)), 0.005);\n"
		"    // check whether current frag pos is in shadow\n"
		"    // float shadow = currentDepth - bias > closestDepth  ? 1.0 : 0.0;\n"
		"    // PCF\n"
		"    float shadow = 0.0;\n"
		"    vec2 texelSize = 1.0 / textureSize(shadowMap, 0);\n"
		"    for(int x = -1; x <= 1; ++x)\n"
		"    {\n"
		"        for(int y = -1; y <= 1; ++y)\n"
		"        {\n"
		"            float pcfDepth = texture(shadowMap, projCoords.xy + vec2(x, y) * texelSize).r; \n"
		"            shadow += currentDepth - bias > pcfDepth  ? 1.0 : 0.0;    \n"
		"        }    \n"
		"    }\n"
		"    shadow /= 9.0;\n"
		"    \n"
		"    // keep the shadow at 0.0 when outside the far_plane region of the light's frustum.\n"
		"    if(projCoords.z > 1.0)\n"
		"        shadow = 0.0;\n"
		"        \n"
		"    return shadow;\n"
		"}\n"
		"\n"
		"void main()\n"
		"{    \n"
		"   vec3 color = texture(diffuseTexture, fs_in.TexCoords).rgb;\n"
		"    vec3 normals = normalize(fs_in.Normal);\n"
		"    vec3 lightColor = vec3(0.3);\n"
		"    // ambient\n"
		"    vec3 ambient = 0.3 * lightColor;\n"
		"    // diffuse\n"
		"    vec3 lightDir = normalize(lightPos - fs_in.FragPos);\n"
		"    float diff = max(dot(lightDir, normals), 0.0);\n"
		"    vec3 diffuse = diff * lightColor;\n"
		"    // specular\n"
		"    vec3 viewDir = normalize(viewPos - fs_in.FragPos);\n"
		"    vec3 reflectDir = reflect(-lightDir, normals);\n"
		"    float spec = 0.0;\n"
		"    vec3 halfwayDir = normalize(lightDir + viewDir);  \n"
		"    spec = pow(max(dot(normals, halfwayDir), 0.0), 64.0);\n"
		"    vec3 specular = spec * lightColor;    \n"
		"    // calculate shadow\n"
		"    float shadow = ShadowCalculation(fs_in.FragPosLightSpace);   \n"
		"    vec3 lighting = (ambient + (1.0 - shadow) * (diffuse + specular)) * color; \n"
		"    \n"
		"    FragColor = vec4(lighting, 1.0);\n"
		"}\n";

	const std::string sourceVShadowMapDepth = "#version 330 core\n"
		"layout (location = 0) in vec3 position;\n"
		"\n"
		"uniform mat4 lightSpaceMatrix;\n"
		"uniform mat4 model;\n"
		"\n"
		"void main()\n"
		"{\n"
		"    gl_Position = lightSpaceMatrix * model * vec4(position, 1.0);\n"
		"}\n";

	const std::string sourceFShadowMapDepth = "#version 330 core\n"
		"\n"
		"void main()\n"
		"{          \n"
		"    // gl_FragDepth = gl_FragCoord.z;\n"
		"}\n";



	// build and compile shaders
	Shader shadowshader(sourceVShadowMap, sourceFShadowMap);
	Shader simpleDepthShader(sourceVShadowMapDepth, sourceFShadowMapDepth);

	Shader shader = Shader(sourceV, sourceF);
	Shader shader2 = Shader(sourceV2, sourceF2);
	Shader shader3 = Shader(sourceV3, sourceF3);
	Shader cubeMapShader = Shader(sourceVCubeMap, sourceFCubeMap);

	// Import objects and create VAO VBO
	char pathCube[] = PATH_TO_OBJECTS "/cube.obj";
	Object cubeMap(pathCube);
	cubeMap.makeObject(cubeMapShader);

	char pathSphere[] = PATH_TO_OBJECTS "/sphere_smooth.obj";
	Object sphere(pathSphere);
	sphere.makeObject(shader2);

	char pathBatter[] = PATH_TO_OBJECTS "/basball.obj";
	Object Batter(pathBatter);
	Batter.makeObject(shadowshader);

	char pathPlane[] = PATH_TO_OBJECTS "/plane.obj";
	Object plane(pathPlane);
	plane.makeObject(shadowshader);


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

	// Ball
	glm::mat4 model = glm::mat4(1.0);
	model = glm::translate(model, glm::vec3(-4, 10, -3.5));
	btRigidBody* bodySphere = createSphere(dynamicsWorld, 1.0, btVector3(-4, 10, -3.5));

	// Batter 
	glm::mat4 model2 = glm::mat4(1.0);
	model2 = glm::translate(model2, glm::vec3(0, 5, 0));
	model2 = glm::scale(model2, glm::vec3(5, 0.1, 3));
	btRigidBody* bodyBatter = createBatter(dynamicsWorld);

	// Ground
	glm::mat4 modelGround = glm::mat4(1.0);
	modelGround = glm::translate(modelGround, glm::vec3(0, 0, 0));
	modelGround = glm::scale(modelGround, glm::vec3(50,0.001, 50));
	btRigidBody* bodyPlaneGround = createPlaneRigidBody(dynamicsWorld, 0, btVector3(0, 0, 0), btVector3(50, 0.001, 50));

	// Light Position
	glm::vec3 light_pos(-1.0f, 15.0f, -5.0f);	// Other position : (1.0, 2.0, 1.5);

	// Initialisation of matrices
	glm::mat4 inverseModel = glm::transpose(glm::inverse(model));
	glm::mat4 view = camera.GetViewMatrix();
	glm::mat4 perspective = camera.GetProjectionMatrix();

	float ambient = 0.1;
	float diffuse = 0.5;
	float specular = 0.8;

	glm::vec3 materialColour = glm::vec3(0.5f, 0.6, 0.8);

	// load textures
	unsigned int metalTexture = loadTexture(PATH_TO_TEXTURE"/metal.png");

	// Initialisation of the shader of the ball 
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



	// Cubemap
	// Initialisation
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

	initialVelocity = bodySphere->getLinearVelocity();
	previousVelocity = initialVelocity;



	// Particle object
	// 1. Triangle vertices
	const float positionsData[9] = {
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


	// Shadows
	// configure depth map FBO
	const unsigned int SHADOW_WIDTH = 1024, SHADOW_HEIGHT = 1024;
	unsigned int depthMapFBO;
	glGenFramebuffers(1, &depthMapFBO);

	// create depth texture
	unsigned int depthMap;
	glGenTextures(1, &depthMap);
	glBindTexture(GL_TEXTURE_2D, depthMap);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	float borderColor[] = { 1.0, 1.0, 1.0, 1.0 };
	glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

	// attach depth texture as FBO's depth buffer
	glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);
	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);


	// Shadow shader configuration
	shadowshader.use();
	shadowshader.setInt("diffuseTexture", 0);
	shadowshader.setInt("shadowMap", 1);



	//Quad on screen 
	//
	float quadVertices[] = { 
		// positions   // texCoords
		-1.0f,  1.0f,  0.0f, 1.0f,
		-1.0f, -1.0f,  0.0f, 0.0f,
		 1.0f, -1.0f,  1.0f, 0.0f,

		-1.0f,  1.0f,  0.0f, 1.0f,
		 1.0f, -1.0f,  1.0f, 0.0f,
		 1.0f,  1.0f,  1.0f, 1.0f
	};

	// screen quad VAO VBO
	unsigned int quadVAO, quadVBO;
	glGenVertexArrays(1, &quadVAO);
	glGenBuffers(1, &quadVBO);
	glBindVertexArray(quadVAO);
	glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));



	// framebuffer configuration
	unsigned int framebuffer;
	glGenFramebuffers(1, &framebuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
	// create a color attachment texture
	unsigned int textureColorbuffer;
	glGenTextures(1, &textureColorbuffer);
	glBindTexture(GL_TEXTURE_2D, textureColorbuffer);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureColorbuffer, 0);
	// create a renderbuffer object for depth and stencil attachment (we won't be sampling these)
	unsigned int rbo;
	glGenRenderbuffers(1, &rbo);
	glBindRenderbuffer(GL_RENDERBUFFER, rbo);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height); // use a single renderbuffer object for both a depth AND stencil buffer.
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo); // now actually attach it
	// now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!";
	glBindFramebuffer(GL_FRAMEBUFFER, 0);


	// shader configuration
	shader3.use();
	shader3.setInt("screenTexture", 0);


	glfwSwapInterval(1);

	// Rendering

	while (!glfwWindowShouldClose(window)) {

		glViewport(0, 0, width, height);

		processInput(window);

		glfwPollEvents();

		double now = glfwGetTime();

		// Update the light position
		light_pos.x = sin(glfwGetTime()) * 1.1f;
		light_pos.y = 15.0 + cos(glfwGetTime()) * 1.0f;
		light_pos.z = cos(glfwGetTime()) * 1.1f;

		// Update physics simulation with Bullet
		dynamicsWorld->stepSimulation(now - lastFrameTime, 10);

		handleBallTransition(bodySphere); // Ball change
		handleThresholdForParticles(bodySphere); // Particles
		updateBodyBatter(bodyBatter); // Batter movements

		// Get the model matrice for the ball from bullet
		btTransform transform;
		bodySphere->getMotionState()->getWorldTransform(transform);
		transform.getOpenGLMatrix(glm::value_ptr(model));
		inverseModel = glm::transpose(glm::inverse(model));


		// Get the model matrice for the batter from bullet
		btTransform transform2;
		bodyBatter->getMotionState()->getWorldTransform(transform2);
		transform2.getOpenGLMatrix(glm::value_ptr(model2));


		// Clear the scene
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);



		// -------first render pass: mirror texture.-------
		// bind to framebuffer and draw to color texture as we normally would from a different point of view.
		glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
		glEnable(GL_DEPTH_TEST); // enable depth testing
		
		// Clear the scene
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glDepthFunc(GL_LESS);

		
		// Bind the data for the particles
		glBindVertexArray(VAO);

		// Bind the data for the texture of the cubeMap
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, cubeMapTexture);
		cubeMapShader.setInteger("cubemapTexture", 0);

		
		// Render the main scene 
		
		// render scene as normal using the generated depth/shadow map  
		shadowshader.use();
		glm::mat4 projection = camera2.GetProjectionMatrix();
		glm::mat4 view = camera2.GetViewMatrix();
		shadowshader.setMat4("projection", projection);
		shadowshader.setMat4("view", view);


		// Bind the data for the texture of the batter and the ground
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, metalTexture);

		// Bind the data for the texture of the depth map
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, depthMap);

		
		// draw batter
		shadowshader.setMat4("model", model2);
		Batter.draw();

		// draw ground 
		shadowshader.setMat4("model", modelGround);
		plane.draw();
		
		// View matrix
		view = camera2.GetViewMatrix();
		perspective = camera2.GetProjectionMatrix();


		// Use the shader Class to send the uniform for the ball
		shader2.use();
		shader2.setMatrix4("M", model);
		shader2.setMatrix4("itM", inverseModel);
		shader2.setMatrix4("V", view);
		shader2.setMatrix4("P", perspective);
		shader2.setVector3f("u_view_pos", camera2.Position);

		auto delta = light_pos + glm::vec3(0.0, 0.0, 2 * std::sin(now));

		// Draw ball
		sphere.draw();


		// Use the shader Class to send the uniform for the others
		shader.use();
		shader.setMatrix4("V", view);
		shader.setMatrix4("P", perspective);

		// draw particles
		for (auto& particle : ballTrailParticles) {

			particle.lifetime -= deltaTime2;
			particle.position += particle.velocity * deltaTime2; 
			particle.velocity.y -= 9.81 * deltaTime2 / 1000;

			glm::mat4 model = glm::translate(glm::mat4(1.0f), particle.position);
			shader.setMatrix4("M", model);
			glDrawArrays(GL_TRIANGLES, 0, 30);

			if (particle.lifetime <= 0.0f) {
				ballTrailParticles.erase(ballTrailParticles.begin(), ballTrailParticles.begin() + 1);
			}
		}
		
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


		glBindFramebuffer(GL_FRAMEBUFFER, 0);



		// -------second render pass: draw as normal-------
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


		// Light & Shadow computations
		// render depth of scene to texture (from light's perspective)
		glm::mat4 lightProjection, lightView;
		glm::mat4 lightSpaceMatrix;
		float near_plane = 1.0f, far_plane = 55.5f;
		lightProjection = glm::ortho(-100.0f, 100.0f, -100.0f, 100.0f, near_plane, far_plane);
		lightView = glm::lookAt(light_pos, glm::vec3(0.0f), glm::vec3(0.0, 1.0, 0.0));
		lightSpaceMatrix = lightProjection * lightView;

		// render scene from light's point of view
		simpleDepthShader.use();
		simpleDepthShader.setMat4("lightSpaceMatrix", lightSpaceMatrix);

		glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
		glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
		glClear(GL_DEPTH_BUFFER_BIT);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, metalTexture);

		// draw ball in depth map 
		simpleDepthShader.setMat4("model", model);
		sphere.draw();

		// draw batter in depth map 
		simpleDepthShader.setMat4("model", model2);
		Batter.draw();

		// draw ground in depth map 
		simpleDepthShader.setMat4("model", modelGround);
		plane.draw();

		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		// reset viewport
		glViewport(0, 0, width, height);


		// Clear the scene
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glDepthFunc(GL_LESS);


		// Bind the data for the particles
		glBindVertexArray(VAO);

		// Bind the data for the texture of the cubeMap
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, cubeMapTexture);
		cubeMapShader.setInteger("cubemapTexture", 0);


		// Render the main scene 

		// render scene as normal using the generated depth/shadow map  
		shadowshader.use();
		projection = camera.GetProjectionMatrix(); //glm::perspective(glm::radians(camera.Zoom), (float)width / (float)height, 0.1f, 100.0f);
		view = camera.GetViewMatrix();
		shadowshader.setMat4("projection", projection);
		shadowshader.setMat4("view", view);

		// set light uniforms
		shadowshader.setVec3("viewPos", camera.Position);
		shadowshader.setVec3("lightPos", light_pos);
		shadowshader.setMat4("lightSpaceMatrix", lightSpaceMatrix);

		// Bind the data for the texture of the batter and the ground
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, metalTexture);

		// Bind the data for the texture of the depth map
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, depthMap);


		// draw batter
		shadowshader.setMat4("model", model2);
		Batter.draw();

		// draw ground 
		shadowshader.setMat4("model", modelGround);
		plane.draw();

		// View matrix
		view = camera.GetViewMatrix();
		perspective = camera.GetProjectionMatrix();


		// Use the shader Class to send the uniform for the ball
		shader2.use();
		shader2.setMatrix4("M", model);
		shader2.setMatrix4("itM", inverseModel);
		shader2.setMatrix4("V", view);
		shader2.setMatrix4("P", perspective);
		shader2.setVector3f("u_view_pos", camera.Position);

		// Draw ball
		sphere.draw();


		// Use the shader Class to send the uniform for the others
		shader.use();
		shader.setMatrix4("V", view);
		shader.setMatrix4("P", perspective);

		// draw particles
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




		// now draw the quad with screen texture
		glDisable(GL_DEPTH_TEST); // disable depth test so screen-space quad isn't discarded due to depth test.
		glViewport(width / 2, height / 2, width / 2, height / 2);


		shader3.use();
		glBindVertexArray(quadVAO);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, textureColorbuffer);	// use the color attachment texture as the texture of the quad plane
		glDrawArrays(GL_TRIANGLES, 0, 6);

		
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
		if (body && body->getMotionState()) {
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

	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
		camera.ProcessKeyboardMovement(LEFT, 0.1);
		camera2.ProcessKeyboardMovement(LEFT, 0.1);
	}

	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
		camera.ProcessKeyboardMovement(RIGHT, 0.1);
		camera2.ProcessKeyboardMovement(RIGHT, 0.1);
	}

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
		camera.ProcessKeyboardMovement(FORWARD, 0.1);
		camera2.ProcessKeyboardMovement(FORWARD, 0.1);
	}

	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
		camera.ProcessKeyboardMovement(BACKWARD, 0.1);
		camera2.ProcessKeyboardMovement(BACKWARD, 0.1);
	}

	if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
		camera.ProcessKeyboardRotation(1, 0.0, 1);
		camera2.ProcessKeyboardRotation(1, 0.0, 1);
	}

	if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
		camera.ProcessKeyboardRotation(-1, 0.0, 1);
		camera2.ProcessKeyboardRotation(-1, 0.0, 1);
	}


	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
		camera.ProcessKeyboardRotation(0.0, 1.0, 1);
		camera2.ProcessKeyboardRotation(0.0, 1.0, 1);
	}

	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
		camera.ProcessKeyboardRotation(0.0, -1.0, 1);
		camera2.ProcessKeyboardRotation(0.0, -1.0, 1);
	}



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


btRigidBody* createPlaneRigidBody(btDynamicsWorld* dynamicsWorld, btScalar mass, const btVector3& position, const btVector3& size) {
	btCollisionShape* colShape = new btBoxShape(size);
	collisionShapes.push_back(colShape);
	
	btTransform mirrorTransform;
	mirrorTransform.setIdentity();
	mirrorTransform.setOrigin(position);

	btVector3 localInertia(0, 0, 0);
	if (mass != 0.f)
		colShape->calculateLocalInertia(mass, localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(mirrorTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
	rbInfo.m_friction = 1.f;
	rbInfo.m_restitution = 0.9f;

	btRigidBody* body = new btRigidBody(rbInfo);
	dynamicsWorld->addRigidBody(body);

	return body;
}


btRigidBody* createBatter(btDynamicsWorld* dynamicsWorld) {
	btCollisionShape* colShape = new btBoxShape(btVector3(btScalar(5.), btScalar(0.1), btScalar(3.)));
	collisionShapes.push_back(colShape);
	btScalar mass(50000);
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0, 5, 0));
	startTransform.setRotation(btQuaternion(0, 0, 0, mass));
	bool isDynamic = (mass != 0.f);
	btVector3 localInertia(100, 0, 100);
	// objects of infinite mass can't move or rotate
	if (isDynamic)
		colShape->calculateLocalInertia(mass, localInertia);

	// create the motion state from the initial transform
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	// create the rigid body construction info using the mass, motion state and shape
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
	rbInfo.m_restitution = 1.0f;

	// create the rigid body
	btRigidBody* bodyBatter = new btRigidBody(rbInfo);

	dynamicsWorld->addRigidBody(bodyBatter);

	return bodyBatter;
	}


void handleBallTransition(btRigidBody* bodySphere) {
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

		previousVelocity = initialVelocity;
		ballTrailParticles.clear();
		ballExceededThreshold = false;
		transitionBall = false;
	}
}

void updateBodyBatter(btRigidBody* bodyBatter) {
	bodyBatter->setLinearVelocity(btVector3(0, 0, 0));
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
}

void handleThresholdForParticles(btRigidBody* bodySphere) {

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
}



// utility function for loading a 2D texture from file
// ---------------------------------------------------
unsigned int loadTexture(char const* path)
{
	unsigned int textureID;
	glGenTextures(1, &textureID);

	int texwidth, texheight, nrComponents;
	unsigned char* data = stbi_load(path, &texwidth, &texheight, &nrComponents, 0);
	if (data)
	{
		GLenum format;
		if (nrComponents == 1)
			format = GL_RED;
		else if (nrComponents == 3)
			format = GL_RGB;
		else if (nrComponents == 4)
			format = GL_RGBA;

		glBindTexture(GL_TEXTURE_2D, textureID);
		glTexImage2D(GL_TEXTURE_2D, 0, format, texwidth, texheight, 0, format, GL_UNSIGNED_BYTE, data);
		glGenerateMipmap(GL_TEXTURE_2D);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, format == GL_RGBA ? GL_CLAMP_TO_EDGE : GL_REPEAT); // for this tutorial: use GL_CLAMP_TO_EDGE to prevent semi-transparent borders. Due to interpolation it takes texels from next repeat 
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, format == GL_RGBA ? GL_CLAMP_TO_EDGE : GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		stbi_image_free(data);
	}
	else
	{
		std::cout << "Texture failed to load at path: " << path << std::endl;
		stbi_image_free(data);
	}

	return textureID;
}

