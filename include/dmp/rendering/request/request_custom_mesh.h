#ifndef DMP_REQUEST_CUSTOM_MESH_H
#define DMP_REQUEST_CUSTOM_MESH_H

#include <dmp/rendering/request/request.h>

#include <vector>
#include <string>

namespace dmp
{
class RequestCustomMesh : public Request
{
public:
  RequestCustomMesh();
  ~RequestCustomMesh() override;

  std::string name;
  std::string frame;
  std::string texture_name;
  std::vector<float> vertex_buffer;
  std::vector<float> normal_buffer;
  std::vector<float> texture_buffer;
  std::vector<float> color_buffer;
  std::vector<int> face_buffer;

private:
};
}

#endif //DMP_REQUEST_CUSTOM_MESH_H
