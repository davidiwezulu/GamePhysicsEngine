# GamePhysicsEngine Documentation

Welcome to the **GamePhysicsEngine** documentation. This document provides detailed information about the structure, usage, and API of the physics engine.

## Table of Contents

1. [Introduction](#introduction)
2. [Installation](#installation)
3. [Using the Engine](#using-the-engine)
4. [API Reference](#api-reference)
5. [Examples](#examples)

## Introduction

The **GamePhysicsEngine** is a 2D/3D physics engine that provides support for rigid body dynamics, soft-body physics, and collision detection. The engine is designed to be lightweight and easily integrable into game engines or simulation environments.

## Installation

To build and install the engine, follow these steps:

```bash
git clone https://github.com/davidiwezulu/GamePhysicsEngine.git
cd GamePhysicsEngine
mkdir build && cd build
cmake ..
make
```
### Using the Engine
To use the engine, you need to create a physics engine instance and add physical bodies to the simulation.

```
#include "PhysicsEngine.h"

PhysicsEngine engine;

RigidBody* body = new RigidBody();
engine.addRigidBody(body);

engine.update(deltaTime);
```
# API Reference

## PhysicsEngine

- `addRigidBody(RigidBody* rigidBody)`: Adds a rigid body to the physics simulation.
- `addSoftBody(SoftBody* softBody)`: Adds a soft body to the physics simulation.
- `update(double deltaTime)`: Updates the physics engine state by a time step.

## RigidBody

- `setMass(double mass)`: Sets the mass of the rigid body.
- `applyForce(const Vector3& force)`: Applies a force to the rigid body.
- `integrate(double deltaTime)`: Integrates the motion of the rigid body over time.

## SoftBody

- `addNode(const Vector3& position)`: Adds a node to the soft body.
- `addLink(int nodeA, int nodeB)`: Adds a link (spring) between two nodes.
- `integrate(double deltaTime)`: Integrates the motion of the soft body over time.

# Examples

For examples on how to use the engine, see the `examples/` directory.

- `RigidBodyDemo.cpp`: Demonstrates rigid body dynamics.
- `SoftBodyDemo.cpp`: Demonstrates soft body physics.
``` 
- 
---

## 🛠️ **Source Code**

Below are all the source and header files with complete expert-level formatting and your name as the author.

### **`include/Vector3.h`**

```cpp
#ifndef VECTOR3_H
#define VECTOR3_H

/**
 * @file Vector3.h
 * @brief Defines the Vector3 class for 3D vector operations.
 * @author David Iwezulu
 */

#include <cmath>

/**
 * @class Vector3
 * @brief Represents a three-dimensional vector and provides common vector operations.
 */
class Vector3 {
public:
    double x; ///< X-component of the vector.
    double y; ///< Y-component of the vector.
    double z; ///< Z-component of the vector.

    /**
     * @brief Default constructor initializes vector to zero.
     */
    Vector3() : x(0), y(0), z(0) {}

    /**
     * @brief Parameterized constructor.
     * @param x X-component.
     * @param y Y-component.
     * @param z Z-component.
     */
    Vector3(double x, double y, double z) : x(x), y(y), z(z) {}

    /**
     * @brief Adds two vectors.
     * @param other Vector to add.
     * @return Resulting vector after addition.
     */
    Vector3 operator+(const Vector3& other) const;

    /**
     * @brief Subtracts two vectors.
     * @param other Vector to subtract.
     * @return Resulting vector after subtraction.
     */
    Vector3 operator-(const Vector3& other) const;

    /**
     * @brief Multiplies vector by a scalar.
     * @param scalar Scalar value to multiply.
     * @return Resulting vector after multiplication.
     */
    Vector3 operator*(double scalar) const;

    /**
     * @brief Calculates the dot product of two vectors.
     * @param other Other vector.
     * @return Dot product.
     */
    double dot(const Vector3& other) const;

    /**
     * @brief Calculates the cross product of two vectors.
     * @param other Other vector.
     * @return Cross product vector.
     */
    Vector3 cross(const Vector3& other) const;

    /**
     * @brief Calculates the magnitude (length) of the vector.
     * @return Magnitude of the vector.
     */
    double magnitude() const;

    /**
     * @brief Normalizes the vector to unit length.
     * @return Normalized vector.
     */
    Vector3 normalize() const;
};

#endif // VECTOR3_H
```

