#ifndef SOFT_BODY_H
#define SOFT_BODY_H

/**
 * @file SoftBody.h
 * @brief Defines the SoftBody class for simulating soft body physics dynamics.
 * @author David Iwezulu
 */

#include <vector>
#include "Vector3.h"

/**
 * @class SoftBody
 * @brief Represents a soft body in the physics simulation.
 */
class SoftBody {
public:
    /**
     * @brief Constructor initializes the soft body with default values.
     */
    SoftBody();

    /**
     * @brief Destructor cleans up any allocated resources.
     */
    ~SoftBody();

    /**
     * @brief Integrates the soft body's motion over time.
     * @param deltaTime Time step in seconds.
     */
    void integrate(double deltaTime);

    /**
     * @brief Adds a node to the soft body.
     * @param position Initial position of the node.
     */
    void addNode(const Vector3& position);

    /**
     * @brief Adds a link (spring) between two nodes.
     * @param nodeA Index of the first node.
     * @param nodeB Index of the second node.
     */
    void addLink(int nodeA, int nodeB);

private:
    struct Node {
        Vector3 position;
        Vector3 velocity;
        Vector3 acceleration;
        double mass;
    };

    struct Link {
        int nodeA;
        int nodeB;
        double restLength;
        double stiffness;
    };

    std::vector<Node> nodes_;   ///< Nodes of the soft body.
    std::vector<Link> links_;   ///< Links (springs) connecting nodes.

    /**
     * @brief Applies forces to the nodes based on the links.
     */
    void applyInternalForces();
};

#endif // SOFT_BODY_H

