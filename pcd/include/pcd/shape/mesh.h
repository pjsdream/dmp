#ifndef DMP_MESH_H
#define DMP_MESH_H

#include <string>

namespace pcd
{
class Mesh
{
public:
  Mesh();

  void loadFromFile(const std::string& filename);

private:
};
}

#endif //DMP_MESH_H
