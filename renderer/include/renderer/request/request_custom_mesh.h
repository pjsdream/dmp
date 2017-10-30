#ifndef DMP_REQUEST_CUSTOM_MESH_H
#define DMP_REQUEST_CUSTOM_MESH_H

#include "request.h"

#include <vector>
#include <string>

#include <Eigen/Dense>

namespace dmp
{
class RequestCustomMesh : public Request
{
public:
  RequestCustomMesh() : Request(Request::Type::CustomMesh)
  {
  }

  ~RequestCustomMesh() override = default;

  bool hasGlobalColor()
  {
    return has_global_color_;
  }

  const Eigen::Vector3f& getGlobalColor()
  {
    return global_color_;
  }

  void setGlobalColor(const Eigen::Vector3f& global_color)
  {
    has_global_color_ = true;
    global_color_ = global_color;
  }

  std::string name;
  std::string frame;
  std::string texture_name;
  std::vector<float> vertex_buffer;
  std::vector<float> normal_buffer;
  std::vector<float> texture_buffer;
  std::vector<float> color_buffer;
  std::vector<int> face_buffer;

  template<typename Archive>
  Archive& serialize(Archive& ar)
  {
    return ar & name & frame & texture_name & vertex_buffer & normal_buffer & texture_buffer & color_buffer
        & face_buffer & has_global_color_ & global_color_;
  }

private:
  bool has_global_color_ = false;
  Eigen::Vector3f global_color_;
};
}

#endif //DMP_REQUEST_CUSTOM_MESH_H
