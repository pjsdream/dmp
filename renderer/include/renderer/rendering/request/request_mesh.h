#ifndef DMP_REQUEST_MESH_H
#define DMP_REQUEST_MESH_H

#include <dmp/rendering/request/request.h>

namespace dmp
{
class RequestMesh : public Request
{
public:
  RequestMesh();
  ~RequestMesh() override = default;

  enum class Action
  {
    Nothing,
    Attach,
    Detach,
  };

  Action action;
  std::string frame;
  std::string filename;
};
}

#endif //DMP_REQUEST_MESH_H
