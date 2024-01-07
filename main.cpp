#include<iostream>
// TODO 
// Framebuffer
// Shadows
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

GLuint mirrorFramebuffer; // Variable globale pour le framebuffer du miroir
GLuint mirrorColorBuffer; // Variable globale pour la texture couleur du miroir
glm::vec3 mirrorPosition = glm::vec3(0.f, 7.f, 16.f); // Position du miroir


struct Particle {
	glm::vec3 position;
	glm::vec3 velocity;  // Initial velocity
	float lifetime;  // Time until the particle disappears
};

std::vector<Particle> ballTrailParticles;
btAlignedObjectArray<btCollisionShape*> collisionShapes;

btRigidBody* createSphere(btDynamicsWorld* dynamicsWorld, btScalar mass, const btVector3& position);
btRigidBody* createPlaneRigidBody(btDynamicsWorld* dynamicsWorld, btScalar mass, const btVector3& position, const btVector3& size);

void initMirrorFramebuffer();
unsigned int loadTexture(const char* path);

void processInput(GLFWwindow* window);

void handleBallTransition(btRigidBody* bodySphere);
void updateBodyBatter(btRigidBody* bodyBatter);
void handleThresholdForParticles(btRigidBody* bodySphere);

void loadCubemapFace(const char* file, const GLenum& targetCube);




void initMirrorFramebuffer() {
	// Créer le framebuffer du miroir
	glGenFramebuffers(1, &mirrorFramebuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, mirrorFramebuffer);

	// Créer la texture couleur attachée au framebuffer
	glGenTextures(1, &mirrorColorBuffer);
	glBindTexture(GL_TEXTURE_2D, mirrorColorBuffer);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width/4, height/4, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Attacher la texture couleur au framebuffer
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mirrorColorBuffer, 0);

	// Créer le tampon de profondeur attaché au framebuffer
	//GLuint mirrorDepthBuffer;
	//glGenRenderbuffers(1, &mirrorDepthBuffer);
	//glBindRenderbuffer(GL_RENDERBUFFER, mirrorDepthBuffer);
	////glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
	//glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, mirrorDepthBuffer);

	// Vérifier si le framebuffer est complet
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		std::cout << "Erreur : Framebuffer du miroir non complet!" << std::endl;
	}

	// Réinitialiser le framebuffer par défaut
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}



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

	char pathPlane2[] = PATH_TO_OBJECTS "/plane.obj";
	Object plane2(pathPlane2);
	plane2.makeObject(shader);




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


	glm::vec3 light_pos(-1.0f, 15.0f, -5.0f);

	//glm::vec3 light_pos = glm::vec3(1.0, 2.0, 1.5);
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
	modelGround = glm::translate(modelGround, glm::vec3(0, 0, 0));
	modelGround = glm::scale(modelGround, glm::vec3(50, 1, 50));

	glm::mat4 modelGround2 = glm::mat4(1.0);
	modelGround2 = glm::translate(modelGround2, glm::vec3(0, 7, 16));
	modelGround2 = glm::scale(modelGround2, glm::vec3(.5, 2, 2));//(-7, 15, 15));

	// One for the ground, one for the mirror
	btRigidBody* bodyPlaneGround = createPlaneRigidBody(dynamicsWorld, 0, btVector3(0, 0, 0), btVector3(50, 1, 50));
	btRigidBody* bodyMirror2 = createPlaneRigidBody(dynamicsWorld, 0, btVector3(0, 7, 16), btVector3(1, 1, 1));;


	//2.b Another one are for the balls of the game
	btRigidBody* bodySphere = createSphere(dynamicsWorld, 1.0, btVector3(-4, 10, -3.5));


	//3.b Another one for the batter
	btRigidBody* bodyBatter;
	{
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

	// Cubemap initialisation
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
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

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












	GLfloat quadVertices[] = {
		// Positions       // Texture Coords
		0.5f,  0.5f, 0.0f, 1.0f,  // Top right
	   -0.5f,  0.5f, 0.0f, 0.0f,  // Top left
	   -0.5f, -0.5f, 0.0f, 0.0f,  // Bottom left
		0.5f, -0.5f, 0.0f, 1.0f   // Bottom right
	};
	GLuint quadVAO, quadVBO;
	glGenVertexArrays(1, &quadVAO);
	glGenBuffers(1, &quadVBO);
	glBindVertexArray(quadVAO);
	glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), (GLvoid*)(2 * sizeof(GLfloat)));
	glEnableVertexAttribArray(1);



	/*
	// Initialization
	initFramebuffer();
	initSecondaryFramebuffer();  // Call this once during initialization

	// Set the callback function for window resize
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	*/

	// Initialiser le framebuffer du miroir
    initMirrorFramebuffer();

	glm::mat4 modelMirror = glm::mat4(1.0);
	modelMirror = glm::translate(modelMirror, mirrorPosition);
	modelMirror = glm::scale(modelMirror, glm::vec3(.5, 2, 2));//(-7, 15, 15));

	glfwSwapInterval(1);

	// Rendering

	while (!glfwWindowShouldClose(window)) {

		processInput(window);

		glfwPollEvents();

		double now = glfwGetTime();

		// Update the light position
		light_pos.x = sin(glfwGetTime()) * 1.1f;
		light_pos.z = cos(glfwGetTime()) * 1.1f;
		//light_pos.y = 5.0 + cos(glfwGetTime()) * 1.0f;

		// Update physics simulation with Bullet
		dynamicsWorld->stepSimulation(now - lastFrameTime, 10);

		handleBallTransition(bodySphere); // Ball change
		handleThresholdForParticles(bodySphere); // Particles
		updateBodyBatter(bodyBatter); // Batter movements

		// Get the model matrice for the ball from bullet
		btTransform transform;
		bodySphere->getMotionState()->getWorldTransform(transform);
		transform.getOpenGLMatrix(glm::value_ptr(model));

		// Get the model matrice for the batter from bullet
		btTransform transform2;
		bodyBatter->getMotionState()->getWorldTransform(transform2);
		transform2.getOpenGLMatrix(glm::value_ptr(model2));


		// Clear the scene
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// render depth of scene to texture (from light's perspective)
		glm::mat4 lightProjection, lightView;
		glm::mat4 lightSpaceMatrix;
		float near_plane = 1.0f, far_plane = 50.5f;
		lightProjection = glm::ortho(-10.0f, 10.0f, -10.0f, 10.0f, near_plane, far_plane);
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
		glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
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
		glm::mat4 projection = camera.GetProjectionMatrix(); //glm::perspective(glm::radians(camera.Zoom), (float)width / (float)height, 0.1f, 100.0f);
		glm::mat4 view = camera.GetViewMatrix();
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
			particle.position += particle.velocity * deltaTime2; // Update position
			particle.velocity.y -= 9.81 * deltaTime2 / 1000; // Apply gravity

			glm::mat4 model = glm::translate(glm::mat4(1.0f), particle.position);
			shader.setMatrix4("M", model);
			glDrawArrays(GL_TRIANGLES, 0, 30);

			if (particle.lifetime <= 0.0f) {
				ballTrailParticles.erase(ballTrailParticles.begin(), ballTrailParticles.begin() + 1);
			}
		}
		
		// draw mirror
		shader.setMatrix4("M", modelGround2);
		plane.draw();


		// draw cubemap
		cubeMapShader.use();
		cubeMapShader.setMatrix4("V", view);
		cubeMapShader.setMatrix4("P", perspective);
		cubeMapShader.setInteger("cubemapTexture", 0);

		glDepthFunc(GL_LEQUAL);  // Set depth function to less than or equal for cubemap
		cubeMap.draw();
		glDepthFunc(GL_LESS);    // Reset depth function to default


		/*
		// Rendering the mirror
		// Set up the mirror view matrix to make it correspond to its point of view

		shader.use();
		//glm::vec3 mirrorPosition = glm::vec3(0.f, 7.f, 16.f); // Position du miroir

		// Inside the mirror framebuffer rendering section
		glm::mat4 mirrorModel = glm::translate(glm::mat4(1.0f), 2.0f * mirrorPosition);
		//glm::mat4 mirrorModel = glm::translate(glm::mat4(1.0f), mirrorPosition);// *glm::scale(glm::mat4(1.0f), glm::vec3(2.0f));
		//shader.setMatrix4("M", mirrorModel); // Use the mirror model matrix
		//shader.setMatrix4("M", mirrorModel);

		//glm::vec3 mirrorCameraPos = mirrorPosition + (camera.Position - mirrorPosition);
		glm::vec3 mirrorCameraPos = 2.0f * mirrorPosition - camera.Position;
		glm::vec3 mirrorTarget = glm::vec3(0.0f, 0.0f, 0.0f); // Point de cible du miroir
		glm::vec3 mirrorUp = glm::vec3(0.0f, -1.0f, 0.0f); // Adjust the "up" vector 
		glm::mat4 mirrorView = glm::lookAt(mirrorCameraPos, mirrorTarget, mirrorUp);

		// Inside the mirror framebuffer rendering section
		glDisable(GL_CULL_FACE); // Disable face culling to render both sides of the geometry


		// Set up the mirror projection (adjust as needed)
		//glm::mat4 mirrorProjection = glm::perspective(glm::radians(45.0f), 1.0f, 0.1f, 100.0f);
		glm::mat4 mirrorProjection =  perspective;
		//glm::mat4 mirrorProjection = glm::mat4(1.0f);


		// Render objects in the mirror
		glBindFramebuffer(GL_FRAMEBUFFER, mirrorFramebuffer);
		glViewport(width / 2, 0, width / 2, height / 2);

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);
		
		// Draw the scene from the mirror's point of view
		shader.use();
		shader.setMatrix4("V", mirrorView);
		shader.setMatrix4("P", mirrorProjection);

		// Draw the batter
		shader.setMatrix4("M", model2);
		.draw();

		// Draw the ground
		shader.setMatrix4("M", modelGround);
		plane.draw();

		// Draw the ground mirror
		shader.setMatrix4("M", modelGround2);
		plane.draw();

		// Draw the cubemap
		cubeMapShader.use();
		cubeMapShader.setMatrix4("V", mirrorView);
		cubeMapShader.setMatrix4("P", mirrorProjection);

		glDepthFunc(GL_LEQUAL);
		//cubeMap.draw();
		glDepthFunc(GL_LESS);

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glViewport(0, 0, width, height);
		glEnable(GL_CULL_FACE);

		// Draw the mirror using the mirror framebuffer texture
		shader.use();
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, mirrorColorBuffer);
		shader.setInteger("mirrorTexture", 1);

		// Draw a quad with the mirror texture
		glBindVertexArray(quadVAO);
		glDrawArrays(GL_TRIANGLES, 0, 6);
		glBindVertexArray(0);

		// Reset to the default framebuffer
		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		// Draw the rest of the scene
		glViewport(0, 0, width, height);

		// Draw the batter
		shader.setMatrix4("V", view);
		shader.setMatrix4("P", perspective);
		shader.setMatrix4("M", model2);
		Batter.draw();

		// Draw the ground
		shader.setMatrix4("M", modelGround);
		plane.draw();

		// Draw the ground mirror
		shader.setMatrix4("M", modelGround2);
		plane.draw();

		// Draw the cubemap
		cubeMapShader.use();
		cubeMapShader.setMatrix4("V", view);
		cubeMapShader.setMatrix4("P", perspective);

		glDepthFunc(GL_LEQUAL);
		//cubeMap.draw();
		glDepthFunc(GL_LESS);



		//// Render mirror-related objects here


		//// Reset framebuffer
		//glBindFramebuffer(GL_FRAMEBUFFER, 0);
		//glViewport(width / 2, 0, width / 2, height / 2);

		////glClearColor(1.f, 1.f, 1.f, 1.f);
		////glClear(GL_COLOR_BUFFER_BIT);
		//shader.use(); // Aply effect

		//glBindVertexArray(quadVAO);
		//glDisable(GL_DEPTH_TEST);
		//glBindTexture(GL_TEXTURE_2D, mirrorFramebuffer);
		//glDrawArrays(GL_TRIANGLES, 0, 6);
		//glBindVertexArray(0);

		//glViewport(0, 0, width, height);
		*/
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

	// Free the memory of the buffer
	glDeleteBuffers(1, &mirrorColorBuffer);

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



// Function to create a static rigid body for the mirror
btRigidBody* createPlaneRigidBody(btDynamicsWorld* dynamicsWorld, btScalar mass, const btVector3& position, const btVector3& size) {
	btCollisionShape* colShape = new btBoxShape(size);
	collisionShapes.push_back(colShape);
	
	// Calculate the transformation for the mirror
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

