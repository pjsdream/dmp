#ifndef DMP_RENDERER_OSTREAM_H
#define DMP_RENDERER_OSTREAM_H

#include <core/comm/publisher.h>
#include <core/utils/timer.h>

namespace dmp
{
class Request;
class RequestMesh;

class RendererOstream
{
public:
  RendererOstream()
      : publisher_("renderer", "127.0.0.1"), created_timer_(0.1)
  {
  }

  template<typename T>
  RendererOstream& operator<<(T& request)
  {
    created_timer_.sleepUntil();
    publisher_ << request.type() << request;
    return *this;
  }

  void flush()
  {
    publisher_ << dmp::Publisher::flush;
  }

private:
  Timer created_timer_;
  Publisher publisher_;
};
}

#endif //DMP_RENDERER_OSTREAM_H
