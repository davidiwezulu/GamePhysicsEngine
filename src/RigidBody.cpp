#include "RigidBody.h"

RigidBody::RigidBody()
    : position_(0, 0, 0), velocity_(0, 0, 0), acceleration_(0, 0, 0), mass_(1.0) {
    // Initialization code
}

RigidBody::~RigidBody() {
    // Cleanup code
}

void RigidBody::integrate(double deltaTime) {
    if (mass_ <= 0.0) return;

    // Update position using Euler integration
    position_ = position_ + velocity_ * deltaTime;

    // Update velocity
    velocity_ = velocity_ + acceleration_ * deltaTime;

    // Reset acceleration
    acceleration_ = Vector3(0, 0, 0);
}

void RigidBody::applyForce(const Vector3& force) {
    if (mass_ <= 0.0) return;

    // F = m * a => a = F / m
    acceleration_ = acceleration_ + force * (1.0 / mass_);
}

void RigidBody::setMass(double mass) {
    if (mass > 0.0) {
        mass_ = mass;
    }
}

Vector3 RigidBody::getPosition() const {
    return position_;
}

Vector3 RigidBody::getVelocity() const {
    return velocity_;
}

void RigidBody::setVelocity(const Vector3& velocity) {
    velocity_ = velocity;
}

