#ifndef DMP_REQUEST_CUSTOM_MESH_H
#define DMP_REQUEST_CUSTOM_MESH_H

#include <dmp/rendering/request/request.h>

#include <vector>
#include <string>

#include <Eigen/Dense>

namespace dmp
{
class RequestCustomMesh : public Request
{
public:
  RequestCustomMesh();
  ~RequestCustomMesh() override = default;

  bool hasGlobalColor();
  const Eigen::Vector3f& getGlobalColor();

  void setGlobalColor(const Eigen::Vector3f& global_color);

  void createCube(Eigen::Vector3d size);
  void createCylinder(double r, double h, int n = 16);
  void createSphere(double r, int subdivision = 3);

  std::string name;
  std::string frame;
  std::string texture_name;
  std::vector<float> vertex_buffer;
  std::vector<float> normal_buffer;
  std::vector<float> texture_buffer;
  std::vector<float> color_buffer;
  std::vector<int> face_buffer;

private:
  void addVertex(float x, float y, float z, float nx, float ny, float nz);
  void addFace(int f0, int f1, int f2);

  void createSphere(double r, Eigen::Vector3d n0, Eigen::Vector3d n1, Eigen::Vector3d n2, int subdivision);

  bool has_global_color_;
  Eigen::Vector3f global_color_;
};
}

#endif //DMP_REQUEST_CUSTOM_MESH_H
