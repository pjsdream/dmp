#ifndef DMP_REQUEST_FRAME_ATTACH_H
#define DMP_REQUEST_FRAME_ATTACH_H

#include "request.h"

namespace dmp
{
class RequestFrameAttach : public Request
{
public:
  RequestFrameAttach()
      : Request(Request::Type::FrameAttach)
  {
  }

  template<typename Archive>
  Archive& serialize(Archive& ar)
  {
    return ar & frame & resource;
  }

  std::string frame;
  std::string resource;

private:
};
}

#endif //DMP_REQUEST_FRAME_ATTACH_H
