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
  struct Flush
  {
  };

  static Flush flush;

  Publisher() = delete;

  Publisher(std::string topic, std::string ip)
      : serializer_(buffer_), topic_(std::move(topic))
  {
    serializer_ << topic_;
    connect(std::move(ip));
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
  Publisher& operator<<(T&& message)
  {
    serializer_ << std::forward<T>(message);
    return *this;
  }

  Publisher& operator<<(Flush flush)
  {
    zmq_socket_->send(buffer_.begin(), buffer_.end());
    buffer_.clear();
    serializer_ << topic_;
  }

private:
  std::string topic_;
  std::unique_ptr<zmq::socket_t> zmq_socket_;

  std::vector<char> buffer_;
  Serializer serializer_;
};
}

#endif //DMP_PUBLISHER_H
