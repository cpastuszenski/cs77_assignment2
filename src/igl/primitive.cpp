#include "primitive.h"

///@file igl/primitive.cpp Primitives. @ingroup igl

/// Computes transformation matrix of TransformedSurface
/// @param transformed The given TransformedSurface for which to compute a transformation matrix
/// @return The transformation matrix
mat4f transformed_matrix(TransformedSurface* transformed) {
    auto m = identity_mat4f;

    // Starting with the identity matrix, compose transformations by matrix multiplication
    // using the 4D transformation matrices described in the lecture notes and the order
    // given in the homework assignment

    // Translation
    auto translation = transformed->translation;
    m *= mat4f(1, 0, 0, translation.x,
               0, 1, 0, translation.y,
               0, 0, 1, translation.z,
               0, 0, 0 , 1);

    // Rotate around z-axis
    auto rot_z = transformed->rotation_euler.z;
    m *= mat4f(cos(rot_z), -sin(rot_z), 0, 0,
               sin(rot_z), cos(rot_z), 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1);

    // Rotate around y-axis
    auto rot_y = transformed->rotation_euler.y;
    m *= mat4f(cos(rot_y), 0, sin(rot_y), 0,
               0, 1, 0, 0,
               -sin(rot_y), 0, cos(rot_y), 0,
               0, 0, 0, 1);

    // Rotate around x-axis
    auto rot_x = transformed->rotation_euler.x;
    m *= mat4f(1, 0, 0, 0,
               0, cos(rot_x), -sin(rot_x), 0,
               0, sin(rot_x), cos(rot_x), 0,
               0, 0, 0, 1);

    // Scaling
    auto scale = transformed->scale;
    m *= mat4f(scale.x, 0, 0, 0,
               0, scale.y, 0, 0,
               0, 0, scale.z, 0,
               0, 0, 0, 1);

    // Return the transformation matrix
    return m;
}

/// Computes inverse transformation matrix of TransformedSurface
/// @param transformed The given TransformedSurface for which to compute an inverse transformation matrix
/// @return The inverse transformation matrix
mat4f transformed_matrix_inv(TransformedSurface* transformed) {
    auto m = identity_mat4f;

    // Starting with the identity matrix, compose transformations by matrix multiplication
    // using the 4D transformation matrices described in the lecture notes and the order opposite
    // given in the homework assignment

    // Scaling
    auto scale = transformed->scale;
    m *= mat4f(1/scale.x, 0, 0, 0,
               0, 1/scale.y, 0, 0,
               0, 0, 1/scale.z, 0,
               0, 0, 0, 1);

    // Rotate around x-axis
    auto rot_x = transformed->rotation_euler.x;
    rot_x *= -1;
    m *= mat4f(1, 0, 0, 0,
               0, cos(rot_x), -sin(rot_x), 0,
               0, sin(rot_x), cos(rot_x), 0,
               0, 0, 0, 1);

    // Rotate around y-axis
    auto rot_y = transformed->rotation_euler.y;
    rot_y *= -1;
    m *= mat4f(cos(rot_y), 0, sin(rot_y), 0,
               0, 1, 0, 0,
               -sin(rot_y), 0, cos(rot_y), 0,
               0, 0, 0, 1);

    // Rotate around z-axis
    auto rot_z = transformed->rotation_euler.z;
    rot_z *= -1;
    m *= mat4f(cos(rot_z), -sin(rot_z), 0, 0,
               sin(rot_z), cos(rot_z), 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1);

    // Translation
    auto translation = transformed->translation;
    m *= mat4f(1, 0, 0, -translation.x,
               0, 1, 0, -translation.y,
               0, 0, 1, -translation.z,
               0, 0, 0 , 1);

    // Return the inverse transformation matrix
    return m;
}

