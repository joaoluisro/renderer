#include <iostream>
#define TINYOBJLOADER_IMPLEMENTATION
#include "tinyobjloader/tiny_obj_loader.h"
#include <vector>
#include "vector3d.h"
#include "triangle.h"
#include <memory>
using namespace std;

int parse_obj(const char *filename, vector<shared_ptr<BaseObject>> &meshdata)
{
    const char *inputfile = filename;
    tinyobj::attrib_t attrib;                   // holds all vertex data
    std::vector<tinyobj::shape_t> shapes;       // array of shapes (faces, etc.)
    std::vector<tinyobj::material_t> materials; // optional .mtl materials
    std::string warn, err;

    // load
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, inputfile);

    if (!warn.empty())
        std::cout << "WARN: " << warn << "\n";
    if (!err.empty())
        std::cerr << "ERR: " << err << "\n";
    if (!ret)
    {
        std::cerr << "Failed to load/parse .obj file\n";
        return 1;
    }

    // iterate shapes
    for (size_t s = 0; s < shapes.size(); s++)
    {
        size_t index_offset = 0;
        //   std::cout << "Shape " << s << " has " << shapes[s].mesh.num_face_vertices.size() << " faces\n";

        // for each face
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++)
        {
            int fv = shapes[s].mesh.num_face_vertices[f];
            std::vector<Vector3D> t_vertices;
            std::vector<Vector3D> t_normals;
            tinyobj::real_t red;
            tinyobj::real_t green;
            tinyobj::real_t blue;
            // for each vertex in the face
            for (int v = 0; v < fv; v++)
            {
                // access index
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

                // retrieve vertex coordinates
                float vx = attrib.vertices[3 * idx.vertex_index + 0];
                float vy = attrib.vertices[3 * idx.vertex_index + 1];
                float vz = attrib.vertices[3 * idx.vertex_index + 2];
                t_vertices.push_back(Vector3D(vx, vy, vz));

                // optional: normals & texcoords
                float nx = 0, ny = 0, nz = 0;
                if (idx.normal_index >= 0)
                {
                    nx = attrib.normals[3 * idx.normal_index + 0];
                    ny = attrib.normals[3 * idx.normal_index + 1];
                    nz = attrib.normals[3 * idx.normal_index + 2];
                }
                t_normals.push_back(Vector3D(nx, ny, nz));

                float tx = 0, ty = 0;
                if (idx.texcoord_index >= 0)
                {
                    tx = attrib.texcoords[2 * idx.texcoord_index + 0];
                    ty = attrib.texcoords[2 * idx.texcoord_index + 1];
                }

                red = attrib.colors[3 * size_t(idx.vertex_index) + 0];
                green = attrib.colors[3 * size_t(idx.vertex_index) + 1];
                blue = attrib.colors[3 * size_t(idx.vertex_index) + 2];

                std::cout << "  v(" << vx << "," << vy << "," << vz << ")"
                          << " n(" << nx << "," << ny << "," << nz << ")"
                          << " uv(" << tx << "," << ty << ")\n";
            }
            meshdata.push_back(make_shared<Triangle>(t_vertices[0], 
                                                    t_vertices[1], 
                                                    t_vertices[2], 
                                                    t_normals[0], 
                                                    t_normals[1], 
                                                    t_normals[2], 
                                                    Color(red,green,blue)));
            index_offset += fv;
        }
    }

    // materials (if any)
    std::cout << "Loaded " << materials.size() << " materials\n";
    for (const auto &m : materials)
    {
        std::cout << " material '" << m.name << "' diffuse_color="
                  << "(" << m.diffuse[0] << "," << m.diffuse[1] << "," << m.diffuse[2] << ")\n";
    }

    return 0;
}
