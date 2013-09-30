#ifndef _PRIMITIVE_H_
#define _PRIMITIVE_H_

#include "node.h"
#include "shape.h"
#include "material.h"

///@file igl/primitive.h Primitives. @ingroup igl
///@defgroup primitive Primitives
///@ingroup igl
///@{

/// Abstract Primitive
struct Primitive : Node {
    frame3f              frame; ///< frame
    Material*            material = nullptr; ///< material
};

/// Group of Primitives
struct PrimitiveGroup : Node {
	vector<Primitive*>       prims; ///< primitives
};

/// Basic Surface
struct Surface : Primitive {
    Shape*               shape = nullptr; ///< shape
};

/// Surface Transformed with arbitrary and animated transformations
struct TransformedSurface : Primitive {
    Shape*              shape = nullptr; ///< shape
    
    frame3f             pivot = identity_frame3f; ///< transformation center and orientation
    
    vec3f               translation = zero3f; ///< translation
    vec3f               rotation_euler = zero3f; ///< rotation along main axis (ZYX order)
    vec3f               scale = one3f; ///< scaling
};


///@name TransformedShape animation support
///@{
mat4f transformed_matrix(TransformedSurface* transformed);
mat4f transformed_matrix_inv(TransformedSurface* transformed);
///@}



///@}

#endif
