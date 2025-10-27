#include <iostream>
#define TINYOBJLOADER_IMPLEMENTATION
#include "tinyobjloader/tiny_obj_loader.h"

#include <vector>
#include <memory>
#include <filesystem>

#include "geometry/vector3d.h"
#include "geometry/triangle.h"

#include "bvh/mesh.h"

using namespace std;

#define DEBUG true

Material parse_material(const vector<tinyobj::material_t> &materials,
                        const int mat_id)
{
    if (mat_id >= 0 && mat_id < (int)materials.size())
    {
        const tinyobj::material_t &m = materials[mat_id];

        auto ambient_color = Color(m.ambient[0],m.ambient[1],m.ambient[2]);
        auto diffuse_color = Color(m.diffuse[0],m.diffuse[1],m.diffuse[2]);
        auto specular_color = Color(m.specular[0],m.specular[1],m.specular[2]);
        double reflectance = (m.specular[0] + m.specular[1] + m.specular[2])/3.0f;
        auto transmittance = Color(m.transmittance[0],m.transmittance[1],m.transmittance[2]);

        Material mat{.ambient=ambient_color,
                     .diffuse=diffuse_color,
                     .specular=specular_color,
                     .spec_exp=m.shininess,
                     .reflectance=reflectance,
                     .transmittance=transmittance,
                     .index_of_ref=m.ior};
        return mat;
    }
    // Fallback to default material
    Material mat{.ambient=Color(0,0,0),
                 .diffuse=Color(0,0,0),
                 .specular=Color(0,0,0),
                 .spec_exp=0.0f,
                 .reflectance=0.0f,
                 .transmittance=Color(0,0,0),
                 .index_of_ref=0.0f};
    return mat;
}

vector<shared_ptr<BaseObject>> parse_faces(const tinyobj::shape_t &shape,
                                           const vector<tinyobj::material_t> &materials,
                                           const tinyobj::attrib_t& attrib)
{
    vector<shared_ptr<BaseObject>> faces;
    size_t index_offset = 0;

    for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); f++)
    {
        int fv = shape.mesh.num_face_vertices[f];

        vector<Vector3D> t_vertices;
        vector<Vector3D> t_normals;
        tinyobj::real_t red;
        tinyobj::real_t green;
        tinyobj::real_t blue;

        for (int v = 0; v < fv; v++)
        {
            // access index
            tinyobj::index_t idx = shape.mesh.indices[index_offset + v];

            // retrieve vertex coordinates
            double vx = attrib.vertices[3 * idx.vertex_index + 0];
            double vy = attrib.vertices[3 * idx.vertex_index + 1];
            double vz = attrib.vertices[3 * idx.vertex_index + 2];
            t_vertices.push_back(Vector3D(vx, vy, vz));
            double nx = 0, ny = 0, nz = 0;
            if (idx.normal_index >= 0)
            {
                nx = attrib.normals[3 * idx.normal_index + 0];
                ny = attrib.normals[3 * idx.normal_index + 1];
                nz = attrib.normals[3 * idx.normal_index + 2];
            }
            t_normals.push_back(Vector3D(nx, ny, nz).normalized());

            // texture coordinates UNUSED
            // double tx = 0, ty = 0;
            // if (idx.texcoord_index >= 0)
            // {
            //  tx = attrib.texcoords[2 * idx.texcoord_index + 0];
            //  ty = attrib.texcoords[2 * idx.texcoord_index + 1];
            // }

            red = attrib.colors[3 * size_t(idx.vertex_index) + 0];
            green = attrib.colors[3 * size_t(idx.vertex_index) + 1];
            blue = attrib.colors[3 * size_t(idx.vertex_index) + 2];
        }
        int material_id = shape.mesh.material_ids[f]; // material index
        auto face_material = parse_material(materials, material_id);

        faces.push_back(make_shared<Triangle>(t_vertices[0],
                                              t_vertices[1],
                                              t_vertices[2],
                                              t_normals[0],
                                              t_normals[1],
                                              t_normals[2],
                                              Color(red,green,blue),
                                              face_material));
        index_offset += fv;
    }
    return faces;
}

int parse_obj(const char *filename,
              vector<shared_ptr<Mesh>> &mesh,
              int leaf_threshold,
              BVHType tree_type)
{
    tinyobj::ObjReaderConfig cfg;

    std::filesystem::path p = filename;

    cfg.mtl_search_path = p.parent_path();
    cfg.triangulate = true;

    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(filename, cfg)) {
        std::cerr << reader.Error() << std::endl;
    }
    if (!reader.Warning().empty()) std::cerr << reader.Warning() << std::endl;

    const auto& materials = reader.GetMaterials();
    const auto& shapes    = reader.GetShapes();
    const auto& attrib    = reader.GetAttrib();

  string warn, err;

  for(size_t s = 0; s < shapes.size(); s++)
  {
    vector<shared_ptr<BaseObject>> faces;

    faces = parse_faces(shapes[s], materials, attrib);
    mesh.push_back(make_shared<Mesh>(faces, false, false, leaf_threshold, tree_type));
  }
  return 0;
}


