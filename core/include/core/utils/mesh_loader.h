#ifndef DMP_MESH_LOADER_H
#define DMP_MESH_LOADER_H

#include <future>
#include <vector>
#include <string>

#include <Eigen/Dense>

namespace dmp
{
struct MeshLoaderRawMesh
{
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
};

class MeshLoader
{
public:

  static std::future<MeshLoaderRawMesh> loadMesh(const std::string& filename);

private:
  static std::string getDirectory(const std::string filename);
};
}

#endif //DMP_MESH_LOADER_H
