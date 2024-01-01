#include<iostream>

//include glad before GLFW to avoid header conflict or define "#define GLFW_INCLUDE_NONE"
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <map>

#include "camera.h"
#include "shader.h"
#include "object.h"
#include "Ball.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>


#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

Ball newBall(glm::vec3 position, btDiscreteDynamicsWorld * dynamicsWorld) {
    // initialize the ball (create rigid body, set initial position, etc.)
    btcollisionshape* colshape = new btsphereshape(btscalar(1.));
    dynamicsworld->addcollisionshape(colshape);

    bttransform starttransform;
    starttransform.setidentity();
    starttransform.setorigin(btvector3(position.x, position.y, position.z)); // origin : btvector3(-4, 4.75, -3.5)

    btscalar mass = 1.f;
    btvector3 localinertia(1, 1, 1);
    colshape->calculatelocalinertia(mass, localinertia);

    btdefaultmotionstate* mymotionstate = new btdefaultmotionstate(starttransform);
    btrigidbody::btrigidbodyconstructioninfo rbinfo(mass, mymotionstate, colshape, localinertia);
    rbinfo.m_friction = 1.f;
    rbinfo.m_restitution = 1.0f;
    body = new btrigidbody(rbinfo);

    dynamicsworld->addrigidbody(body);
}

void Ball::update(Object body) {
    // Update the ball's physics state
    // dynamicsWorld->stepSimulation(1.0f / 60.0f, 10);
    // Update the model matrix based on the rigid body's position
    std::cout << "\r in update ball ";
    btTransform transform;
    body->getMotionState()->getWorldTransform(transform);
    transform.getOpenGLMatrix(glm::value_ptr(model));
}

void Ball::draw(Shader& shader) {
    // Draw the ball using the provided shader
    // Use the shader Class to send the uniform
    std::cout << "\r in draw ball ";

    shader.use();
    shader.setMatrix4("M", model);
    sphere.draw();
}