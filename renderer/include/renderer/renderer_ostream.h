#ifndef DMP_RENDERER_OSTREAM_H
#define DMP_RENDERER_OSTREAM_H

#include <core/comm/publisher.h>

namespace dmp
{
class Request;
class RequestMesh;

class RendererOstream
{
public:
  RendererOstream();

  RendererOstream& operator<<(RequestMesh& mesh);

private:
  Publisher publisher_;

  std::shared_ptr<Context> context_;
};

extern RendererOstream rout;
}

#endif //DMP_RENDERER_OSTREAM_H
