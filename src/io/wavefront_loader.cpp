#include <iostream>
#define TINYOBJLOADER_IMPLEMENTATION
#include "tinyobjloader/tiny_obj_loader.h"

#include "io/wavefront_loader.h"

Material parseMaterial(const vector<tinyobj::material_t> &materials,
                       const int mat_id)
{
    if (mat_id >= 0 && mat_id < (int)materials.size())
    {
        const tinyobj::material_t &m = materials[mat_id];

        auto ambient_color = Spectrum(m.ambient[0],m.ambient[1],m.ambient[2]);
        auto diffuse_color = Spectrum(m.diffuse[0],m.diffuse[1],m.diffuse[2]);
        auto specular_color = Spectrum(m.specular[0],m.specular[1],m.specular[2]);
        auto transmittance = Spectrum(m.transmittance[0],m.transmittance[1],m.transmittance[2]);
        auto emittance = Spectrum(m.emission[0], m.emission[1], m.emission[2]);
        IllumType illum;

        switch (m.illum)
        {
        case 0:
            illum = IllumType::OPAQUE;
            break;
        case 1:
            illum = IllumType::SPECULAR;
            break;
        case 2:
            illum = IllumType::SPECULAR_DIFFUSE;
            break;
        case 3:
            illum = IllumType::TRANSPARENT;
            break;
        case 4:
            illum = IllumType::MIRROR;
            break;
        case 5:
            illum = IllumType::AREA_LIGHT;
            break;
        default:
            illum = IllumType::OPAQUE;
            break;
        }
        Material mat{.ambient=ambient_color,
                     .diffuse=diffuse_color,
                     .specular=specular_color,
                     .spec_exp=m.shininess,
                     .transmittance=transmittance,
                     .emittance=emittance,
                     .transparency=m.dissolve,
                     .index_of_ref=m.ior,
                     .illum=illum,
                     .is_transparent=m.illum == 3,
                     .is_lightsource=m.illum == 5};
        return mat;
    }
    // Fallback to default material
    Material mat{.ambient=Spectrum(0,0,0),
                 .diffuse=Spectrum(0,0,0),
                 .specular=Spectrum(0,0,0),
                 .spec_exp=0.0f,
                 .transmittance=Spectrum(0,0,0),
                 .emittance=Spectrum(0,0,0),
                 .transparency=0.0f,
                 .index_of_ref=0.0f,
                 .illum=IllumType::OPAQUE,
                 .is_transparent=false,
                 .is_lightsource=false};
    return mat;
}

vector<shared_ptr<Face>> parseFaces(const tinyobj::shape_t &shape,
                                    const vector<tinyobj::material_t> &materials,
                                    const tinyobj::attrib_t& attrib)
{
    vector<shared_ptr<Face>> faces;
    size_t index_offset = 0;

    for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); f++)
    {
        int fv = shape.mesh.num_face_vertices[f];

        vector<vec3> t_vertices;
        vector<vec3> t_normals;
        tinyobj::real_t red;
        tinyobj::real_t green;
        tinyobj::real_t blue;

        for (int v = 0; v < fv; v++)
        {
            // access index
            tinyobj::index_t idx = shape.mesh.indices[index_offset + v];

            // retrieve vertex coordinates
            float vx = attrib.vertices[3 * idx.vertex_index + 0];
            float vy = attrib.vertices[3 * idx.vertex_index + 1];
            float vz = attrib.vertices[3 * idx.vertex_index + 2];
            t_vertices.push_back(vec3(vx, vy, vz));
            float nx = 0, ny = 0, nz = 0;
            if (idx.normal_index >= 0)
            {
                nx = attrib.normals[3 * idx.normal_index + 0];
                ny = attrib.normals[3 * idx.normal_index + 1];
                nz = attrib.normals[3 * idx.normal_index + 2];
            }
            t_normals.push_back(vec3(nx, ny, nz));

            // texture coordinates UNUSED
            // float tx = 0, ty = 0;
            // if (idx.texcoord_index >= 0)
            // {
            //  tx = attrib.texcoords[2 * idx.texcoord_index + 0];
            //  ty = attrib.texcoords[2 * idx.texcoord_index + 1];
            // }

            red = attrib.colors[3 * size_t(idx.vertex_index) + 0];
            green = attrib.colors[3 * size_t(idx.vertex_index) + 1];
            blue = attrib.colors[3 * size_t(idx.vertex_index) + 2];
        }

        faces.push_back(make_shared<Triangle>(t_vertices[0],
                                              t_vertices[1],
                                              t_vertices[2],
                                              t_normals[0],
                                              t_normals[1],
                                              t_normals[2],
                                              Spectrum(red,green,blue)));
        index_offset += fv;
    }
    return faces;
}

int parseWavefrontFile(const char *filename,
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

    cout << shapes.size();
    for(size_t s = 0; s < shapes.size(); s++)
    {
        vector<shared_ptr<Face>> faces;

        faces = parseFaces(shapes[s], materials, attrib);
        int material_id = shapes[s].mesh.material_ids[0];
        auto material = parseMaterial(materials, material_id);

        mesh.push_back(make_shared<Mesh>(faces, leaf_threshold, tree_type, material));
    }
  return 0;
}


