#include <iostream>
#include "PhysicsEngine.h"

/**
 * @file RigidBodyDemo.cpp
 * @brief Demonstrates rigid body physics simulation using the PhysicsEngine.
 * @author David Iwezulu
 */

int main() {
    PhysicsEngine physicsEngine;

    // Create rigid bodies
    RigidBody* body1 = new RigidBody();
    body1->setMass(2.0);
    body1->applyForce(Vector3(10, 0, 0));

    RigidBody* body2 = new RigidBody();
    body2->setMass(1.0);
    body2->applyForce(Vector3(-5, 0, 0));

    // Add bodies to the engine
    physicsEngine.addRigidBody(body1);
    physicsEngine.addRigidBody(body2);

    // Simulate for 5 seconds
    double totalTime = 5.0;
    double deltaTime = 0.016; // ~60 FPS

    for (double time = 0; time < totalTime; time += deltaTime) {
        physicsEngine.update(deltaTime);

        // Output positions
        std::cout << "Time: " << time << "s\n";
        std::cout << "Body1 Position: " << body1->getPosition().x << ", " << body1->getPosition().y << ", " << body1->getPosition().z << "\n";
        std::cout << "Body2 Position: " << body2->getPosition().x << ", " << body2->getPosition().y << ", " << body2->getPosition().z << "\n";
    }

    // Clean up
    delete body1;
    delete body2;

    return 0;
}

