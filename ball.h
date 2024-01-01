#ifndef BALL_H
#define BALL_H

#include<iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>

#include <btBulletDynamicsCommon.h>
#include "shader.h"
#include "object.h"

class Ball {
public:
	Ball newBall(glm::vec3 position, btDiscreteDynamicsWorld * dynamicsWorld);
	void update(Object body);
	void draw(Shader& shader);

private:
	btRigidBody* body;
	glm::mat4 model;  // Add a model matrix for rendering
	// ??? Add necessary member variables (e.g., model, rigid body, etc.)
};

#endif