#ifndef _MATERIAL_H_
#define _MATERIAL_H_

#include "node.h"

///@file igl/material.h Materials. @ingroup igl
///@defgroup material Materials
///@ingroup igl
///@{

/// Abstract Material
struct Material : Node {
};

/// Lambert Material
struct Lambert : Material {
    vec3f        diffuse = vec3f(0.75,0.75,0.75); ///< diffuse color
};

/// Phong Material
struct Phong : Material {
	vec3f        diffuse = vec3f(0.75,0.75,0.75); ///< diffuse color
    vec3f        specular = vec3f(0.25,0.25,0.25); ///< specular color
    float        exponent =  10; ///< specular exponent
    vec3f        reflection = zero3f; ///< reflection color
    
    bool use_reflected = false; ///< use reflected or bisector
};

///@name eval interface
///@{

/// evalute perturbed shading frame
inline frame3f material_shading_frame(Material* material, const frame3f& frame, const vec2f& texcoord) {
    return frame;
}

/// evaluate produce of BRDF and cosine
inline vec3f material_brdfcos(Material* material, const frame3f& frame, const vec3f& wi, const vec3f& wo, const vec2f& texcoord) {
    auto sf = material_shading_frame(material, frame, texcoord);
    if(is<Lambert>(material)) {
        auto lambert = cast<Lambert>(material);
        if(dot(wi,sf.z) <= 0 or dot(wo,sf.z) <= 0) return zero3f;
        auto tkd = lambert->diffuse;
        return tkd * abs(dot(wi,sf.z));
    }
    else if(is<Phong>(material)) {
        auto phong = cast<Phong>(material);
        if(dot(wi,sf.z) <= 0 or dot(wo,sf.z) <= 0) return zero3f;
        auto tkd = phong->diffuse;
        auto tks = phong->specular;
        if(phong->use_reflected) {
            vec3f wr = reflect(-wi,sf.z);
            return tkd + tks*pow(max(dot(wo,wr),0.0f),phong->exponent);
        } else {
            vec3f wh = normalize(wi+wo);
            return (tkd + tks*pow(max(dot(sf.z,wh),0.0f),phong->exponent)) * abs(dot(wi,sf.z));
        }
    }
    else { not_implemented_error(); return zero3f; }
}

/// evaluate diffuse albedo (color)
inline vec3f material_diffuse_albedo(Material* material, const vec2f& texcoord) {
    if(is<Lambert>(material)) return cast<Lambert>(material)->diffuse;
    else if(is<Phong>(material)) return cast<Phong>(material)->diffuse;
    else { not_implemented_error(); return zero3f; }
}

/// evaluate color of mirror reflection (zero if not reflections)
inline vec3f material_reflection(Material* material, const frame3f& frame, const vec3f& w, const vec2f& texcoord) {
    auto sf = material_shading_frame(material, frame, texcoord);
    if(is<Lambert>(material)) return zero3f;
    else if(is<Phong>(material)) {
        auto phong = cast<Phong>(material);
        if(dot(w,sf.z) <= 0) return zero3f;
        auto tkr = phong->reflection;
        return tkr;
    }
    else { not_implemented_error(); return zero3f; }
}

/// material average color for interactive drawing
inline vec3f material_color(Material* material) {
    if(is<Lambert>(material)) return cast<Lambert>(material)->diffuse;
    else if(is<Phong>(material)) return cast<Phong>(material)->diffuse;
    else { not_implemented_error(); return zero3f; }
}
///@}

///@}

#endif
