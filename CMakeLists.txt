cmake_minimum_required(VERSION 3.10)
project(GamePhysicsEngine VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(include)

# Source files
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Create the library
add_library(GamePhysicsEngine STATIC ${SOURCES})

# Examples
add_executable(RigidBodyDemo examples/RigidBodyDemo.cpp)
target_link_libraries(RigidBodyDemo GamePhysicsEngine)

add_executable(SoftBodyDemo examples/SoftBodyDemo.cpp)
target_link_libraries(SoftBodyDemo GamePhysicsEngine)

# Tests
enable_testing()
add_executable(PhysicsEngineTests tests/PhysicsEngineTests.cpp)
target_link_libraries(PhysicsEngineTests GamePhysicsEngine)
add_test(NAME PhysicsEngineTests COMMAND PhysicsEngineTests)

