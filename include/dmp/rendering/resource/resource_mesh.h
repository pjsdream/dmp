#ifndef DMP_RESOURCE_MESH_H
#define DMP_RESOURCE_MESH_H

#include <dmp/rendering/resource/resource.h>

#include <string>
#include <future>

#include <Eigen/Dense>

namespace dmp
{
class ResourceTexture;
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

  bool hasTexture();
  std::shared_ptr<ResourceTexture> getTexture();

  bool hasColor();

  bool hasGlobalColor();
  const Eigen::Vector3f& getGlobalColor();

  void draw();

private:
  enum class ColorOption
  {
    Texture,
    Color,
    GlobalColor,
  };

  ColorOption color_option_;

  struct RawMesh;
  RawMesh asyncLoadMesh(std::string filename);
  std::future<RawMesh> future_raw_mesh_;

  void prepareGlBuffers();

  bool ready_rendering_;
  GLuint vao_;
  std::vector<GLuint> vbos_;

  int num_faces_;

  std::shared_ptr<ResourceTexture> texture_;
  Eigen::Vector3f global_color_;
};
}

#endif //DMP_RESOURCE_MESH_H
