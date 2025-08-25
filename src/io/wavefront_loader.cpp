#include <iostream>
#define TINYOBJLOADER_IMPLEMENTATION
#include "tinyobjloader/tiny_obj_loader.h"

#include <vector>
#include <memory>

#include "geometry/vector3d.h"
#include "geometry/triangle.h"

#include "bvh/mesh.h"

using namespace std;

#define DEBUG false

int parse_obj(const char *filename, vector<shared_ptr<Mesh>> &mesh, int leaf_threshold, BVHType treeType)
{
  const char *inputfile = filename;
  tinyobj::attrib_t attrib;                  
  std::vector<tinyobj::shape_t> shapes;      
  std::vector<tinyobj::material_t> materials; 
  std::string warn, err;
  bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials,&warn, &err, inputfile);

  if (!ret)
  {
    std::cerr << "Failed to load/parse .obj file\n";
    return 0;
  }
  int total_faces = 0;
  for (size_t s = 0; s < shapes.size(); s++)
  {
    total_faces += shapes[s].mesh.num_face_vertices.size();

    size_t index_offset = 0;
    if(DEBUG) std::cout << "Shape " << shapes[s].name << " has " << shapes[s].mesh.num_face_vertices.size() << " faces\n";
    
    vector<shared_ptr<BaseObject>> faces;

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
        double vx = attrib.vertices[3 * idx.vertex_index + 0];
        double vy = attrib.vertices[3 * idx.vertex_index + 1];
        double vz = attrib.vertices[3 * idx.vertex_index + 2];
        t_vertices.push_back(Vector3D(vx, vy, vz));

        // optional: normals & texcoords
        double nx = 0, ny = 0, nz = 0;
        if (idx.normal_index >= 0)
        {
          nx = attrib.normals[3 * idx.normal_index + 0];
          ny = attrib.normals[3 * idx.normal_index + 1];
          nz = attrib.normals[3 * idx.normal_index + 2];
        }
        t_normals.push_back(Vector3D(nx, ny, nz));

        double tx = 0, ty = 0;
        if (idx.texcoord_index >= 0)
        {
          tx = attrib.texcoords[2 * idx.texcoord_index + 0];
          ty = attrib.texcoords[2 * idx.texcoord_index + 1];
        }

        red = attrib.colors[3 * size_t(idx.vertex_index) + 0];
        green = attrib.colors[3 * size_t(idx.vertex_index) + 1];
        blue = attrib.colors[3 * size_t(idx.vertex_index) + 2];
      }
      Material face_material;
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

    mesh.push_back(make_shared<Mesh>(faces, false, false, leaf_threshold, treeType));
  }
  return total_faces;
}

// TODO: Add material support

// int mat_id = shapes[s].mesh.material_ids[0];
// Material face_material;
// // lookup material (if any)
// if (mat_id >= 0 && mat_id < (int)materials.size()) {
//   const tinyobj::material_t &m = materials[mat_id];
//   std::cout << " material '" << m.name << "' diffuse_color="
//       << "(" << m.diffuse[0] << "," << m.diffuse[1] << "," << m.diffuse[2] << ")\n";
//   std::cout << " material '" << m.name << "' specular_color="
//             << "(" << m.specular[0] << "," << m.specular[1] << "," << m.specular[2] << ")\n";
//   auto ambient_color = Color(m.ambient[0],m.ambient[1],m.ambient[2]);
//   auto diffuse_color = Color(m.diffuse[0],m.diffuse[1],m.diffuse[2]);
//   auto specular_color = Color(m.specular[0],m.specular[1],m.specular[2]);
//   double reflectance = (m.specular[0] + m.specular[1] + m.specular[2])/3.0f;
//   auto transmittance = Color(m.transmittance[0],m.transmittance[1],m.transmittance[2]);

//   Material mat{.ambient=ambient_color,
//                 .diffuse=diffuse_color,
//                 .specular=specular_color,
//                 .spec_exp=m.shininess,
//                 .reflectance=reflectance,
//                 .transmittance=transmittance,
//                 .index_of_ref=m.ior};
//   face_material = mat;
//   // mat.name, mat.diffuse[], mat.specular[], mat.shininess, etc.
//   } else {
//     Material mat{.ambient=Color(0,0,0),
//                 .diffuse=Color(0,0,0),
//                 .specular=Color(0,0,0),
//                 .spec_exp=0.0f,
//                 .reflectance=0.0f,
//                 .transmittance=Color(0,0,0),
//                 .index_of_ref=0.0f};
//     face_material = mat;
//   }