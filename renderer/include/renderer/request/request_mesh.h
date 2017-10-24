#ifndef DMP_REQUEST_MESH_H
#define DMP_REQUEST_MESH_H

#include "request.h"
#include <core/comm/serializer.h>

#include <vector>
#include <zmq.hpp>

namespace dmp
{
class RequestMesh : public Request
{
public:
  RequestMesh() : Request(Request::Type::Mesh)
  {
  }

  ~RequestMesh() override = default;

  std::string name;
  std::string filename;

  template<typename Archive>
  Archive& serialize(Archive& ar)
  {
    return ar & type() & name & filename;
  }
};
}

#endif //DMP_REQUEST_MESH_H
