#include "io/parsing.h"

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

Scene* scene_file(const char *filename, const int width, const int height, const float fov)
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
        std::string wavefront_filepath, tmp;
        std::getline(file, tmp);

        wavefront_filepath = tmp.substr(tmp.find(DEFAULT_DELIMITER) + 1, tmp.length());
        return wavefront_filepath;
    };

    auto parse_camera = [&file, &parse_point, &width, &height, &fov]()
    {
        Vector3D origin, look_at;
        std::string tmp, origin_as_str, look_at_as_str, fov_as_str;

        std::getline(file, tmp);
        origin_as_str = tmp.substr(tmp.find(DEFAULT_DELIMITER), tmp.length());
        origin = parse_point(origin_as_str);

        std::getline(file, tmp);
        look_at_as_str = tmp.substr(tmp.find(DEFAULT_DELIMITER), tmp.length());
        look_at = parse_point(look_at_as_str);

        return new Camera(origin, look_at, fov, height, width);
    };

    auto parse_lighting = [&file, &parse_point]()
    {
        Vector3D pos, c_tmp;
        Color color;
        std::string pos_as_str, type_as_str, color_as_str, tmp;
        std::getline(file, tmp);
        // Since we only allow point lights, do nothing
        type_as_str = tmp;

        std::getline(file, tmp);
        pos_as_str = tmp.substr(tmp.find(DEFAULT_DELIMITER), tmp.length());
        pos = parse_point(pos_as_str);

        std::getline(file, tmp);
        color_as_str = tmp.substr(tmp.find(DEFAULT_DELIMITER), tmp.length());
        c_tmp = parse_point(color_as_str);
        color.r = c_tmp.x;
        color.g = c_tmp.y;
        color.b = c_tmp.z;

        return Light(pos.to_blender(), color);
    };

    std::string line;

    std::string wavefront_filepath;
    Camera *camera;
    std::vector<std::shared_ptr<Light>> lights;
    std::vector<std::shared_ptr<Mesh>> meshes;

    while(std::getline(file, line))
    {
        if(line == WAVEFRONT_SECTION_SIGNATURE)
        {
            wavefront_filepath = parse_wavefront();
        }
        else if(line == CAMERA_SECTION_SIGNATURE)
        {
            camera = parse_camera();
        }
        else if(line == LIGHTING_SECTION_SIGNATURE)
        {
            Light l = parse_lighting();
            lights.push_back(make_shared<Light>(l));
        }
    }
    cout << "Rendering: " <<wavefront_filepath << "\n";
    cout << " Light info : ";
    for(auto &l : lights)
    {
        l->color.info();
    }
    parse_obj(wavefront_filepath.c_str(), meshes, 16, BVHType::MIDPOINT);
    camera->zoom(2.5);
    Scene *s = new Scene(*camera, meshes, lights);
    int count = 0;
    for(auto m : meshes)
    {
        std::cout<<"Mesh " << count << " \n";
        std::cout << "Color : ";
        m->faces[0]->get_color().info();
        std::cout << "Ambient : ";
        m->faces[0]->material().ambient.info();

        std::cout << "Diffuse : ";
        m->faces[0]->material().diffuse.info();

        std::cout << "Specular :";
        m->faces[0]->material().specular.info();
        std::cout<<"------------------------\n";
        count++;
    }
    return s;
}
}
