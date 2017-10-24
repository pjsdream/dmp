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
  Publisher() = delete;

  explicit Publisher(std::string topic)
  : topic_(std::move(topic))
  {
  }

  Publisher(std::string topic, std::string ip)
  : topic_(std::move(topic))
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
    size_evaluator << topic_ << message;

    ZmqSerializer serializer(size_evaluator.size());
    serializer << topic_ << message;
    zmq_socket_->send(serializer.getMessage());
  }

private:
  std::string topic_;
  std::unique_ptr<zmq::socket_t> zmq_socket_;
};
}

#endif //DMP_PUBLISHER_H
