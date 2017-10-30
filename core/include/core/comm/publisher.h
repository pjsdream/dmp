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
  struct Flush {};
  static Flush flush;

  Publisher() = delete;

  explicit Publisher(std::string topic)
  : topic_(std::move(topic))
  {
    prepareSerializer();
  }

  Publisher(std::string topic, std::string ip)
  : topic_(std::move(topic))
  {
    connect(ip);
    prepareSerializer();
  }

  void connect(std::string ip)
  {
    zmq_socket_ = Context::getInstance()->createPublisherSocket(ip);
  }

  /*
  template<typename T>
  void publish(T& message)
  {
    ZmqSerializerSizeEvaluator size_evaluator;
    size_evaluator << topic_ << message;

    ZmqSerializer serializer(size_evaluator.size());
    serializer << topic_ << message;
    zmq_socket_->send(serializer.getMessage());
  }
   */

  template<typename T>
  Publisher& operator << (T&& message)
  {
    return *this;
  }

private:
  static void prepareSerializer()
  {
    // TODO
    serializer_
  }

  std::string topic_;
  std::unique_ptr<zmq::socket_t> zmq_socket_;

  Serializer serializer_;
};
}

#endif //DMP_PUBLISHER_H
