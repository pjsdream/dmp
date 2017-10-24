#ifndef DMP_RAW_MESH_H
#define DMP_RAW_MESH_H

#include <vector>

#include <Eigen/Dense>

namespace dmp
{
class RawMesh
{
public:
  std::vector<float> vertex_buffer;
  std::vector<float> normal_buffer;
  std::vector<float> texture_buffer;
  std::vector<float> color_buffer;
  std::vector<int> face_buffer;

  bool has_global_color;
  Eigen::Vector3f global_color;

  std::string texture_filename;

  struct Material
  {
    float ambient[3];
    float diffuse[3];
    float specular[3];
  };
  Material material;

private:
};
}

#endif //DMP_RAW_MESH_H
