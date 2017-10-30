#ifndef DMP_REQUEST_FRAME_H
#define DMP_REQUEST_FRAME_H

#include "request.h"

#include <Eigen/Dense>

namespace dmp
{
class RequestFrame : public Request
{
public:
  RequestFrame()
      : Request(Request::Type::Frame), transform(Eigen::Affine3d::Identity())
  {
  }

  ~RequestFrame() override = default;

  enum class Action
  {
    Nothing,
    Set,
    Delete,
  };

  template<typename Archive>
  Archive& serialize(Archive& ar)
  {
    return ar & action & name & parent & transform;
  }

  Action action;
  std::string name;
  std::string parent;
  Eigen::Affine3d transform;
};
}

#endif //DMP_REQUEST_FRAME_H
