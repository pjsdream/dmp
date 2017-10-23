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
    // Fill up the message buffer
    std::vector<char> buffer;
    Serializer serializer(buffer);
    serializer << message;

    // Send through zmq message
    // TODO: remove unnecessary copy from byte buffer to zmq message
    zmq::message_t zmq_message(buffer.data(), sizeof(buffer[0]) * buffer.size());
    zmq_socket_->send(zmq_message);
  }

private:
  std::unique_ptr<zmq::socket_t> zmq_socket_;
};
}

#endif //DMP_PUBLISHER_H