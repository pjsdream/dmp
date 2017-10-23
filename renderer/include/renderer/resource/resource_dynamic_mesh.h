#ifndef DMP_RESOURCE_DYNAMIC_MESH_H
#define DMP_RESOURCE_DYNAMIC_MESH_H

#include <dmp/rendering/resource/resource.h>

namespace dmp
{
class ResourceDynamicMesh : public Resource
{
public:
  ResourceDynamicMesh() = delete;
  explicit ResourceDynamicMesh(const std::shared_ptr<GlFunctions>& gl);
  ~ResourceDynamicMesh() override;

  void updateVertexBuffer(std::vector<float>&& b);
  void updateNormalBuffer(std::vector<float>&& b);
  void updateTextureBuffer(std::vector<float>&& b);
  void updateColorBuffer(std::vector<float>&& b);
  void updateFaceBuffer(std::vector<int>&& b);

  void draw();

private:
  static const int INITIAL_BUFFER_ELEMENTS = 1024;

  void allocateGlBuffers();

  GLuint vao_;
  std::vector<GLuint> vbos_;

  int num_faces_;
};
}

#endif //DMP_RESOURCE_DYNAMIC_MESH_H
