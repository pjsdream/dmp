#ifndef DMP_PUBLISHER_H
#define DMP_PUBLISHER_H

#include <memory>

#include "context.h"
#include "serializer.h"

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
  void publish(const T& message)
  {
    // Send through zmq message
    zmq_socket_->send(message.createMessage());
  }

private:
  std::unique_ptr<zmq::socket_t> zmq_socket_;
};
}

#endif //DMP_PUBLISHER_H
