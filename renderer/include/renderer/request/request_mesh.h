#ifndef DMP_REQUEST_MESH_H
#define DMP_REQUEST_MESH_H

#include "request.h"
#include <renderer/request/request.h>
#include <core/comm/serializer.h>

#include <vector>
#include <zmq.hpp>

namespace dmp
{
class RequestMesh : public Request
{
public:
  RequestMesh()
      : action(Action::Nothing)
  {
  }

  ~RequestMesh() override = default;

  enum class Action : unsigned char
  {
    Nothing,
    Attach,
    Detach,
  };

  Action action;
  std::string frame;
  std::string filename;

  template<typename Archive>
  Archive& serialize(Archive& ar)
  {
    return ar & action & frame & filename;
  }
};
}

#endif //DMP_REQUEST_MESH_H
