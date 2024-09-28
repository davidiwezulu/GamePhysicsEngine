#include <iostream>
#include "PhysicsEngine.h"

/**
 * @file SoftBodyDemo.cpp
 * @brief Demonstrates soft body physics simulation using the PhysicsEngine.
 * @author David Iwezulu
 */

int main() {
    PhysicsEngine physicsEngine;

    // Create a soft body (e.g., a simple square cloth)
    SoftBody* cloth = new SoftBody();

    // Add nodes
    cloth->addNode(Vector3(0, 0, 0));
    cloth->addNode(Vector3(1, 0, 0));
    cloth->addNode(Vector3(0, 1, 0));
    cloth->addNode(Vector3(1, 1, 0));

    // Add links between nodes
    cloth->addLink(0, 1);
    cloth->addLink(1, 3);
    cloth->addLink(3, 2);
    cloth->addLink(2, 0);
    cloth->addLink(0, 3);
    cloth->addLink(1, 2);

    // Add soft body to the engine
    physicsEngine.addSoftBody(cloth);

    // Simulate for 5 seconds
    double totalTime = 5.0;
    double deltaTime = 0.016; // ~60 FPS

    for (double time = 0; time < totalTime; time += deltaTime) {
        physicsEngine.update(deltaTime);

        // Output node positions
        // ...
    }

    // Clean up
    delete cloth;

    return 0;
}

