#include <renderer/renderer_ostream.h>
#include <renderer/request/request_mesh.h>

#include <thread>

namespace dmp
{
RendererOstream rout;

RendererOstream::RendererOstream()
{
  using namespace std::chrono_literals;

  context_ = Context::getInstance();
  publisher_.connect("127.0.0.1");

  std::this_thread::sleep_for(100ms);
}

RendererOstream& RendererOstream::operator<<(RequestMesh& mesh)
{
  publisher_.publish(mesh);
  return *this;
}
}
