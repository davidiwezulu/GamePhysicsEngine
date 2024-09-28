# C++ GamePhysicsEngine

A robust and efficient C++ physics engine for 2D/3D simulations, featuring collision detection, rigid body dynamics, and soft-body physics. Designed for easy integration into game engines and simulation projects.

## Author

**David Iwezulu**

## Features

- **Rigid Body Dynamics**: Simulate solid objects with mass, velocity, and acceleration.
- **Soft Body Physics**: Simulate deformable objects like cloth and jelly.
- **Collision Detection**: Detect and resolve collisions between objects.
- **Extensible Design**: Easily extend functionalities or integrate into existing projects.
- **Cross-Platform**: Compatible with Windows, macOS, and Linux.

## Getting Started

### Prerequisites

- C++17 or later
- CMake 3.10 or later
- A compatible C++ compiler (GCC, Clang, MSVC)

### Building the Engine

```bash
git clone https://github.com/davidiwezulu/GamePhysicsEngine.git
cd GamePhysicsEngine
mkdir build && cd build
cmake ..
make
```
### Running Examples
``` 
./examples/RigidBodyDemo
./examples/SoftBodyDemo
```
### Usage
Include the engine in your project:
``` 
#include "PhysicsEngine.h"

// Create a physics engine instance
PhysicsEngine physicsEngine;

// Add physics bodies
RigidBody* rigidBody = new RigidBody();
physicsEngine.addRigidBody(rigidBody);

// Run the simulation
double deltaTime = 0.016; // 60 FPS
physicsEngine.update(deltaTime);
```
### Documentation
Detailed documentation is available in the docs/ directory or can be generated using Doxygen.

### Contributing
Contributions are welcome! Please read the CONTRIBUTING.md for guidelines.

### License
This project is licensed under the MIT License - see the LICENSE file for details.

### Contact
For any inquiries, please contact David Iwezulu at email@example.com.
``` 

---

## 📄 **CONTRIBUTING.md**

```markdown
# Contributing to GamePhysicsEngine

Thank you for considering contributing to the GamePhysicsEngine! Here are some guidelines to help you get started.

## How to Contribute

1. Fork the repository.
2. Create a new branch (`git checkout -b feature/NewFeature`).
3. Commit your changes (`git commit -am 'Add new feature'`).
4. Push to the branch (`git push origin feature/NewFeature`).
5. Create a new Pull Request.

## Coding Standards

- Follow the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).
- Write doc blocks for all public classes and functions.
- Write unit tests for new features.

## Reporting Issues

Please use the [GitHub Issues](https://github.com/davidiwezulu/GamePhysicsEngine/issues) to report bugs or request features.

## Author

**David Iwezulu**
```

