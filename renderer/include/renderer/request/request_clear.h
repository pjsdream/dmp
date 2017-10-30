#ifndef DMP_REQUEST_CLEAR_H
#define DMP_REQUEST_CLEAR_H

#include "request.h"

namespace dmp
{
class RequestClear : public Request
{
public:
  RequestClear() : Request(Request::Type::Clear)
  {
  }

  template<typename Archive>
  Archive& serialize(Archive& ar)
  {
    return ar;
  }
};
}

#endif //DMP_REQUEST_CLEAR_H
