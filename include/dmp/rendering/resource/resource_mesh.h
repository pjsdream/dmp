#ifndef DMP_RESOURCE_MESH_H
#define DMP_RESOURCE_MESH_H

#include <dmp/rendering/resource/resource.h>

#include <string>
#include <future>

namespace dmp
{
class ResourceMesh : public Resource
{
public:
  ResourceMesh() = delete;
  explicit ResourceMesh(const std::shared_ptr<GlFunctions>& gl, const std::string& fileneame);
  ~ResourceMesh() override;

  ResourceMesh(const ResourceMesh& rhs) = delete;
  ResourceMesh& operator=(const ResourceMesh& rhs) = delete;

  ResourceMesh(ResourceMesh&& rhs) = delete;
  ResourceMesh& operator=(ResourceMesh&& rhs) = delete;

  void draw();

private:
  struct RawMesh;

  RawMesh asyncLoadMesh(std::string filename);
  std::future<RawMesh> future_raw_mesh_;

  void prepareGlBuffers();

  bool ready_rendering_;
  GLuint vao_;
  std::vector<GLuint> vbos_;

  int num_faces_;
};
}

#endif //DMP_RESOURCE_MESH_H
