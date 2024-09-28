#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

/**
 * @file CollisionDetection.h
 * @brief Defines the CollisionDetection class for handling collisions between physics bodies.
 * @author David Iwezulu
 */

#include <vector>
#include "RigidBody.h"
#include "SoftBody.h"

/**
 * @class CollisionDetection
 * @brief Handles collision detection and response between physics bodies.
 */
class CollisionDetection {
public:
    /**
     * @brief Detects collisions between rigid bodies and soft bodies.
     * @param rigidBodies List of rigid bodies.
     * @param softBodies List of soft bodies.
     */
    void detectCollisions(const std::vector<RigidBody*>& rigidBodies,
                          const std::vector<SoftBody*>& softBodies);

    /**
     * @brief Resolves detected collisions.
     */
    void resolveCollisions();

private:
    struct CollisionPair {
        RigidBody* rigidBodyA;
        RigidBody* rigidBodyB;
        Vector3 contactPoint;
        Vector3 normal;
    };

    std::vector<CollisionPair> collisions_; ///< Detected collisions.

    /**
     * @brief Detects collisions between two rigid bodies.
     * @param bodyA First rigid body.
     * @param bodyB Second rigid body.
     */
    void detectRigidBodyCollision(RigidBody* bodyA, RigidBody* bodyB);
};

#endif // COLLISION_DETECTION_H

