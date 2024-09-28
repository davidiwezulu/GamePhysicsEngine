#include "PhysicsEngine.h"

PhysicsEngine::PhysicsEngine() {
    // Initialization code
}

PhysicsEngine::~PhysicsEngine() {
    // Cleanup code
    for (auto body : rigidBodies_) {
        delete body;
    }
    for (auto body : softBodies_) {
        delete body;
    }
}

void PhysicsEngine::addRigidBody(RigidBody* rigidBody) {
    rigidBodies_.push_back(rigidBody);
}

void PhysicsEngine::addSoftBody(SoftBody* softBody) {
    softBodies_.push_back(softBody);
}

void PhysicsEngine::update(double deltaTime) {
    integrate(deltaTime);
    handleCollisions();
}

void PhysicsEngine::integrate(double deltaTime) {
    for (auto& body : rigidBodies_) {
        body->integrate(deltaTime);
    }
    for (auto& body : softBodies_) {
        body->integrate(deltaTime);
    }
}

void PhysicsEngine::handleCollisions() {
    collisionDetection_.detectCollisions(rigidBodies_, softBodies_);
    collisionDetection_.resolveCollisions();
}

