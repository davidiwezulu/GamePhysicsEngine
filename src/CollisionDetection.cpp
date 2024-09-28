#include "CollisionDetection.h"

void CollisionDetection::detectCollisions(const std::vector<RigidBody*>& rigidBodies,
                                          const std::vector<SoftBody*>& softBodies) {
    // Clear previous collisions
    collisions_.clear();

    // Detect collisions between rigid bodies
    for (size_t i = 0; i < rigidBodies.size(); ++i) {
        for (size_t j = i + 1; j < rigidBodies.size(); ++j) {
            detectRigidBodyCollision(rigidBodies[i], rigidBodies[j]);
        }
    }

    // Additional collision detection between rigid bodies and soft bodies...
}

void CollisionDetection::detectRigidBodyCollision(RigidBody* bodyA, RigidBody* bodyB) {
    // Simplified collision detection (e.g., sphere-sphere collision)
    Vector3 delta = bodyB->getPosition() - bodyA->getPosition();
    double distance = delta.magnitude();
    double radiusA = 1.0; // Assume unit sphere for simplicity
    double radiusB = 1.0;

    if (distance < (radiusA + radiusB)) {
        // Collision detected
        CollisionPair pair;
        pair.rigidBodyA = bodyA;
        pair.rigidBodyB = bodyB;
        pair.contactPoint = bodyA->getPosition() + delta * (radiusA / (radiusA + radiusB));
        pair.normal = delta.normalize();
        collisions_.push_back(pair);
    }
}

void CollisionDetection::resolveCollisions() {
    for (const auto& collision : collisions_) {
        // Simplified collision response
        RigidBody* bodyA = collision.rigidBodyA;
        RigidBody* bodyB = collision.rigidBodyB;
        Vector3 normal = collision.normal;

        // Swap velocities (perfectly elastic collision)
        Vector3 tempVelocity = bodyA->getVelocity();
        bodyA->setVelocity(bodyB->getVelocity());
        bodyB->setVelocity(tempVelocity);
    }
}

