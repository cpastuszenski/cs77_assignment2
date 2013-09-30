#ifndef _LIGHT_H_
#define _LIGHT_H_

#include "node.h"
#include "shape.h"

///@file igl/light.h Lights. @ingroup igl
///@defgroup light Lights
///@ingroup igl
///@{

/// Abstract Light
struct Light : Node {
    frame3f             frame = identity_frame3f; ///< frame
};

/// Point Light at the origin
struct PointLight : Light {
    vec3f               intensity = one3f; ///< intensity
};

/// Directional Light along z
struct DirectionalLight : Light {
    vec3f               intensity = one3f; ///< intensity
};

/// Group of Lights
struct LightGroup : Node {
    vector<Light*>      lights;
};

///@name sample interface
/// requsted number of shadow rays
inline int light_shadow_nsamples(Light* light) {
    return 1;
}

/// Shadow Sample
struct ShadowSample {
    vec3f           radiance; ///< light radiance
    ray3f           ray; ///< shadow ray
};

/// shadow ray and radiance for light center
inline ShadowSample light_shadow_sample(Light* light, const vec3f& p) {
    auto pl = transform_point_inverse(light->frame, p);
    ShadowSample ss;
    if(is<PointLight>(light)) {
        ss.ray = ray3f::segment(pl,zero3f);
        ss.radiance = cast<PointLight>(light)->intensity / lengthSqr(pl);
    }
    else if(is<DirectionalLight>(light)) {
        ss.ray = ray3f(pl,-z3f);
        ss.radiance = cast<DirectionalLight>(light)->intensity;
    }
    else { not_implemented_error(); }
    ss.ray = transform_ray(light->frame, ss.ray);
    return ss;
}

/// shadow ray and radiance for light location
inline ShadowSample light_shadow_sample(Light* light, const vec3f& p, const vec2f& uv) {
    auto pl = transform_point_inverse(light->frame, p);
    ShadowSample ss;
    return light_shadow_sample(light, p);
    ss.ray = transform_ray(light->frame, ss.ray);
    return ss;
}
///@}

///@name frame manipulation
/// orients a light
inline void light_lookat(Light* light, const vec3f& eye, const vec3f& center, const vec3f& up) {
    light->frame.o = eye;
    light->frame.z = normalize(center-eye);
    light->frame.y = up;
    light->frame = orthonormalize(light->frame);
}
///@}

///@}

#endif
