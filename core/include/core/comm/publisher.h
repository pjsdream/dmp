#ifndef DMP_PUBLISHER_H
#define DMP_PUBLISHER_H

#include <memory>

#include "context.h"
#include "serializer.h"

namespace dmp
{
template<typename T>
class Publisher
{
public:
  Publisher() = delete;

  explicit Publisher(Context& context, std::string topic)
      : zmq_socket_(context.createPublisherSocket())
  {
  }

  void publish(const T& message)
  {
    // Fill up the message buffer
    std::vector<char> buffer;
    Serializer serializer(buffer);
    serializer << message;

    // Send through zmq message
    // TODO: remove unnecessary copy from byte buffer to zmq message
    zmq::message_t zmq_message(buffer.size());
    memcpy(zmq_message.data(), buffer.data(), sizeof(buffer[0]) * buffer.size());
    zmq_socket_.send(zmq_message);
  }

private:
  zmq::socket_t zmq_socket_;
};
}

#endif //DMP_PUBLISHER_H
