#include "SoftBody.h"

SoftBody::SoftBody() {
    // Initialization code
}

SoftBody::~SoftBody() {
    // Cleanup code
}

void SoftBody::integrate(double deltaTime) {
    applyInternalForces();

    // Integrate nodes
    for (auto& node : nodes_) {
        // Update position
        node.position = node.position + node.velocity * deltaTime;

        // Update velocity
        node.velocity = node.velocity + node.acceleration * deltaTime;

        // Reset acceleration
        node.acceleration = Vector3(0, 0, 0);
    }
}

void SoftBody::addNode(const Vector3& position) {
    Node node;
    node.position = position;
    node.velocity = Vector3(0, 0, 0);
    node.acceleration = Vector3(0, 0, 0);
    node.mass = 1.0; // Default mass
    nodes_.push_back(node);
}

void SoftBody::addLink(int nodeA, int nodeB) {
    if (nodeA >= nodes_.size() || nodeB >= nodes_.size()) return;

    Link link;
    link.nodeA = nodeA;
    link.nodeB = nodeB;
    link.restLength = (nodes_[nodeA].position - nodes_[nodeB].position).magnitude();
    link.stiffness = 100.0; // Default stiffness
    links_.push_back(link);
}

void SoftBody::applyInternalForces() {
    for (const auto& link : links_) {
        Node& nodeA = nodes_[link.nodeA];
        Node& nodeB = nodes_[link.nodeB];

        Vector3 delta = nodeB.position - nodeA.position;
        double currentLength = delta.magnitude();
        double displacement = currentLength - link.restLength;

        // Hooke's Law: F = -k * x
        Vector3 force = delta.normalize() * (-link.stiffness * displacement);

        // Apply forces to nodes
        nodeA.acceleration = nodeA.acceleration + force * (1.0 / nodeA.mass);
        nodeB.acceleration = nodeB.acceleration - force * (1.0 / nodeB.mass);
    }
}

