#include <renderer/renderer_ostream.h>
#include <renderer/request/request_mesh.h>

#include <thread>

namespace dmp
{
RendererOstream::RendererOstream()
    : publisher_("renderer", "127.0.0.1"), created_timer_(0.1)
{
}

void RendererOstream::flush()
{
  publisher_ << dmp::Publisher::flush;
}
}
