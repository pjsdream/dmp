#include <renderer/renderer_ostream.h>

namespace dmp
{
RendererOstream rout;

RendererOstream::RendererOstream()
{
}

RendererOstream::~RendererOstream()
{
}

RendererOstream& RendererOstream::operator<<(RequestMesh& mesh)
{
  return *this;
}
}
