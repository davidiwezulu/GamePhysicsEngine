#ifndef RIGID_BODY_H
#define RIGID_BODY_H

/**
 * @file RigidBody.h
 * @brief Defines the RigidBody class for simulating rigid body physics dynamics.
 * @author David Iwezulu
 */

#include "Vector3.h"

/**
 * @class RigidBody
 * @brief Represents a rigid body in the physics simulation.
 */
class RigidBody {
public:
    /**
     * @brief Constructor initializes the rigid body with default values.
     */
    RigidBody();

    /**
     * @brief Destructor cleans up any allocated resources.
     */
    ~RigidBody();

    /**
     * @brief Integrates the rigid body's motion over time.
     * @param deltaTime Time step in seconds.
     */
    void integrate(double deltaTime);

    /**
     * @brief Applies a force to the rigid body.
     * @param force Force vector to apply.
     */
    void applyForce(const Vector3& force);

    /**
     * @brief Sets the mass of the rigid body.
     * @param mass Mass value (must be positive).
     */
    void setMass(double mass);

    /**
     * @brief Gets the current position of the rigid body.
     * @return Current position vector.
     */
    Vector3 getPosition() const;

    /**
     * @brief Gets the current velocity of the rigid body.
     * @return Current velocity vector.
     */
    Vector3 getVelocity() const;

    /**
     * @brief Sets the velocity of the rigid body.
     * @param velocity Velocity vector.
     */
    void setVelocity(const Vector3& velocity);

private:
    Vector3 position_;      ///< Position of the rigid body.
    Vector3 velocity_;      ///< Linear velocity.
    Vector3 acceleration_;  ///< Linear acceleration.
    double mass_;           ///< Mass of the rigid body.
};

#endif // RIGID_BODY_H

