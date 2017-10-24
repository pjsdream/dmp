#ifndef DMP_PUBLISHER_H
#define DMP_PUBLISHER_H

#include <memory>

#include "context.h"
#include "serializer.h"
#include "zmq_serializer.h"

namespace dmp
{
class Publisher
{
public:
  Publisher() = default;

  explicit Publisher(std::string ip)
  {
    connect(ip);
  }

  void connect(std::string ip)
  {
    zmq_socket_ = Context::getInstance()->createPublisherSocket(ip);
  }

  template<typename T>
  void publish(T& message)
  {
    ZmqSerializerSizeEvaluator size_evaluator;
    size_evaluator << message;

    ZmqSerializer serializer(size_evaluator.size());
    serializer << message;
    zmq_socket_->send(serializer.getMessage());
  }

private:
  std::unique_ptr<zmq::socket_t> zmq_socket_;
};
}

#endif //DMP_PUBLISHER_H
