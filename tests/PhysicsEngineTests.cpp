#include <cassert>
#include <iostream>
#include "PhysicsEngine.h"

/**
 * @file PhysicsEngineTests.cpp
 * @brief Unit tests for the PhysicsEngine components.
 * @author David Iwezulu
 */

int main() {
    // Test Vector3 operations
    Vector3 v1(1, 2, 3);
    Vector3 v2(4, -5, 6);

    Vector3 v_add = v1 + v2;
    assert(v_add.x == 5 && v_add.y == -3 && v_add.z == 9);

    Vector3 v_sub = v1 - v2;
    assert(v_sub.x == -3 && v_sub.y == 7 && v_sub.z == -3);

    double dot_product = v1.dot(v2);
    assert(dot_product == (1*4 + 2*-5 + 3*6));

    // Test RigidBody integration
    RigidBody body;
    body.setMass(1.0);
    body.applyForce(Vector3(10, 0, 0));
    body.integrate(1.0);
    assert(body.getPosition().x == 5.0);

    // Additional tests...

    std::cout << "All tests passed successfully.\n";
    return 0;
}

