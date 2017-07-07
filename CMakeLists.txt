cmake_minimum_required(VERSION 3.6)
project(Fluids)

set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_FLAGS "-O3")
#set(CMAKE_EXE_LINKER_FLAGS -static)


FIND_PACKAGE(PNG REQUIRED)
INCLUDE_DIRECTORIES(${PNG_INCLUDE_DIRS})
LINK_LIBRARIES(${PNG_LIBRARIES})

FIND_PACKAGE(GLUT REQUIRED)
INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIRS})
LINK_LIBRARIES(${GLUT_LIBRARIES})

FIND_PACKAGE(OpenGl REQUIRED)
INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIRS})
LINK_LIBRARIES(${OPENGL_LIBRARIES})

include_directories(lib/eigen)
include_directories(${EIGEN_EXTRA_INCS})
add_definitions(${EIGEN_EXTRA_DEFS})

INCLUDE_DIRECTORIES(include)

set(SOURCE_FILES
        include/gfx/config.h
        include/gfx/config-osx.h
        include/gfx/geom3d.h
        include/gfx/geom4d.h
        include/gfx/gfx.h
        include/gfx/mat2.h
        include/gfx/mat3.h
        include/gfx/mat4.h
        include/gfx/quat.h
        include/gfx/symmat2.h
        include/gfx/symmat3.h
        include/gfx/symmat4.h
        include/gfx/vec2.h
        include/gfx/vec3.h
        include/gfx/vec4.h
        Particle.cpp
        Particle.h
        Fluids.cpp
        solvers/Euler.cpp
        solvers/Euler.h
        solvers/Solver.h
        solvers/ConstraintSolver.cpp
        solvers/ConstraintSolver.h
        View.cpp
        View.h
        solvers/Midpoint.cpp
        solvers/Midpoint.h
        solvers/RungeKutta.h
        solvers/RungeKutta.cpp
        System.cpp
        System.h
        SystemBuilder.cpp
        SystemBuilder.h
        constraints/Constraint.h
        constraints/RodConstraint.cpp
        constraints/RodConstraint.h
        constraints/CircularWireConstraint.cpp
        constraints/CircularWireConstraint.h
        forces/Force.cpp
        forces/Force.h
        Kernels.cpp
        Kernels.h
        forces/DirectionalForce.cpp
        forces/DirectionalForce.h
        forces/DragForce.cpp
        forces/DragForce.h
        fields/DensityField.cpp
        fields/DensityField.h
        forces/PressureForce.cpp
        forces/PressureForce.h
        fields/PressureField.cpp
        fields/PressureField.h
        forces/ViscosityForce.cpp
        forces/ViscosityForce.h
        forces/SurfaceForce.h
        forces/SurfaceForce.cpp
        forces/SpringForce.h
        forces/SpringForce.cpp
        forces/AngularSpringForce.h
        forces/AngularSpringForce.cpp
        fields/ColorField.cpp
        fields/ColorField.h
        data/UniformGrid.cpp
        data/UniformGrid.h
        RigidBody.cpp
        RigidBody.h
        MarchingCubes.cpp
        MarchingCubes.h
        solvers/LeapFrog.cpp
        solvers/LeapFrog.h
        Contact.h
        Object.cpp
        Object.h
        Contact.cpp parameters.h)


add_executable(Fluids ${SOURCE_FILES})