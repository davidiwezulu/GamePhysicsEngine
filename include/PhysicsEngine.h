#ifndef PHYSICS_ENGINE_H
#define PHYSICS_ENGINE_H

/**
 * @file PhysicsEngine.h
 * @brief Defines the PhysicsEngine class for managing the physics simulation.
 * @author David Iwezulu
 */

#include <vector>
#include "RigidBody.h"
#include "SoftBody.h"
#include "CollisionDetection.h"

/**
 * @class PhysicsEngine
 * @brief Core physics engine managing simulation updates and physics objects.
 */
class PhysicsEngine {
public:
    /**
     * @brief Constructor for PhysicsEngine.
     */
    PhysicsEngine();

    /**
     * @brief Destructor for PhysicsEngine.
     */
    ~PhysicsEngine();

    /**
     * @brief Adds a rigid body to the simulation.
     * @param rigidBody Pointer to the RigidBody object.
     */
    void addRigidBody(RigidBody* rigidBody);

    /**
     * @brief Adds a soft body to the simulation.
     * @param softBody Pointer to the SoftBody object.
     */
    void addSoftBody(SoftBody* softBody);

    /**
     * @brief Updates the physics simulation by a time step.
     * @param deltaTime Time step in seconds.
     */
    void update(double deltaTime);

private:
    std::vector<RigidBody*> rigidBodies_; ///< List of rigid bodies.
    std::vector<SoftBody*> softBodies_;   ///< List of soft bodies.
    CollisionDetection collisionDetection_; ///< Collision detection system.

    /**
     * @brief Integrates physics bodies over time.
     * @param deltaTime Time step in seconds.
     */
    void integrate(double deltaTime);

    /**
     * @brief Handles collision detection and response.
     */
    void handleCollisions();
};

#endif // PHYSICS_ENGINE_H

