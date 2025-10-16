#include "io/parsing.h"
#include <cstring>

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

// Available only for UNIX systems.
// TODO: Turn it into an OS agnostic API.

namespace Parse{

Scene* scene_file(const char *filename)
{
    std::ifstream file(filename);
    if (!file) return nullptr;

    auto parse_point = [](std::string point_as_string)
    {
        auto ret = point_as_string;
        ret.pop_back();
        ret.erase(0,1);
        auto comma_position = ret.find(DEFAULT_FLOAT_SEPARATOR);
        auto x_as_str = ret.substr(1,comma_position-1);
        ret = ret.substr(comma_position + 1, ret.length());

        comma_position = ret.find(DEFAULT_FLOAT_SEPARATOR);
        auto y_as_str = ret.substr(0,comma_position);
        ret = ret.substr(comma_position + 1, ret.length());

        auto z_as_str = ret;
        return Vector3D(std::stod(x_as_str), std::stod(y_as_str), std::stod(z_as_str));
    };

    auto parse_wavefront = [&file]()
    {
        std::string wavefront_filepath;
        std::getline(file, wavefront_filepath);
        constexpr int sep_pos = std::strlen(WAVEFRONT_PATH_KEY);
        wavefront_filepath.erase(0, sep_pos);
        return wavefront_filepath;
    };

    auto parse_camera = [&file, &parse_point]()
    {
        Vector3D origin, look_at;
        std::string tmp, origin_as_str, look_at_as_str;

        std::getline(file, tmp);
        origin_as_str = tmp.substr(tmp.find(DEFAULT_DELIMITER), tmp.length());
        origin = parse_point(origin_as_str);

        std::getline(file, tmp);
        look_at_as_str = tmp.substr(tmp.find(DEFAULT_DELIMITER), tmp.length());
        look_at = parse_point(look_at_as_str);

        return new Camera(origin, look_at, fov, height, width);
    };

    auto parse_lighting = [](std::string &line, std::string &file)
    {

    };

    std::string line;

    Camera *scene_camera;
    std::string wavefront_filepath;
    std::vector<Light> scene_lights;
    while(std::getline(file, line))
    {
        if(line == WAVEFRONT_SECTION_SIGNATURE)
        {
            wavefront_filepath = parse_wavefront();
        }
        else if(line == CAMERA_SECTION_SIGNATURE)
        {
            scene_camera = parse_camera();
        }
        else if(line == LIGHTING_SECTION_SIGNATURE)
        {
            Light l = parse_lighting();
            scene_lights.push_back(l);
        }
    }
}

}

