#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#define DEBUG false
#define GO_THROUGH_TRANSPARENT false

#define BSDF_SAMPLING_ONLY false
#define LIGHT_SAMPLING_ONLY false

#define UNIFORM_SAMPLING_ONLY false
#define COSINE_SAMPLING_ONLY false

#define MIS_DISABLED false

enum IllumType{
    OPAQUE = 0,
    SPECULAR = 1,
    SPECULAR_DIFFUSE = 2,
    TRANSPARENT = 3,
    MIRROR = 4,
    AREA_LIGHT = 5
};

#define EPSILON 1e-5
#define WORLD_UP vec3(0.0f, 1.0f, 0.0f)

#define MAX_BOUNCE 10
#define MAX_VAL 1e+9
#define BACKGROUND_COLOR Spectrum(0,0,0)
#define NULL_RADIANCE Spectrum(0,0,0)

#define LIGHT_INTENSITY 10.0f
#define MIRROR_FALLOFF 0.8f
#define WEIGHT_DIRECT_LIGHT 0.5f
#define WEIGHT_INDIRECT_LIGHT 0.5f

#define RR_START_DEPTH 3
#define RR_TERMINATION_PROB 0.8f

#define WAVEFRONT_SECTION_SIGNATURE "[FILEPATH]"
#define CAMERA_SECTION_SIGNATURE    "[CAMERA]"
#define LIGHTING_SECTION_SIGNATURE  "[LIGHT]"

#define DEFAULT_DELIMITER "="
#define DEFAULT_FLOAT_SEPARATOR ","

#define WAVEFRONT_PATH_KEY "inpath="

#define ORIGIN_KEY         "origin="
#define LOOK_AT_KEY        "look_at="
#define FOV_KEY            "fov="

#define LIGHT_TYPE_KEY     "type="
#define LIGHT_POSITION_KEY "position="

#endif // DEFINITIONS_H
