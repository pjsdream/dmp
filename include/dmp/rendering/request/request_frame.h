#ifndef DMP_REQUEST_FRAME_H
#define DMP_REQUEST_FRAME_H

#include <dmp/rendering/request/request.h>

#include <Eigen/Dense>

namespace dmp
{
class RequestFrame : public Request
{
public:
  RequestFrame();
  ~RequestFrame() override;

  enum class Action
  {
    Nothing,
    Set,
    Delete,
  };

  Action action;
  std::string name;
  std::string parent;
  Eigen::Affine3d transform;
};
}

#endif //DMP_REQUEST_FRAME_H
