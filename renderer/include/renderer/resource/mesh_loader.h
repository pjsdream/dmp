#ifndef DMP_MESH_LOADER_H
#define DMP_MESH_LOADER_H

#include <future>
#include <vector>
#include <string>

#include <Eigen/Dense>

namespace dmp
{
class RawMesh;

class MeshLoader
{
public:

  static std::future<RawMesh> loadMesh(const std::string& filename);

private:
  static std::string getDirectory(const std::string filename);
};
}

#endif //DMP_MESH_LOADER_H
