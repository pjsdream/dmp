#ifndef DMP_SUBSCRIBER_H
#define DMP_SUBSCRIBER_H

#include "context.h"
#include "deserializer.h"

#include <type_traits>
#include <memory>
#include <iostream>

namespace dmp
{
class Subscriber
{
public:
  Subscriber() = delete;

  Subscriber(std::string ip, std::string topic)
      : zmq_socket_(Context::getInstance()->createSubscriberSocket(ip))
  {
  }

  // Returns true when it received message successfully
  template<typename T>
  bool receive(T& result)
  {
    // Receive message through zmq
    zmq::message_t zmq_message;
    auto received = zmq_socket_->recv(&zmq_message, ZMQ_NOBLOCK);
    if (!received)
      return false;

    // Deserialize
    // TODO: remove unnecessary buffer creation and copy
    std::vector<char> buffer((char *)zmq_message.data(), (char *)zmq_message.data() + zmq_message.size());
    Deserializer deserializer(buffer);
    deserializer >> result;
    return true;
  }

private:
  std::unique_ptr<zmq::socket_t> zmq_socket_;
};
}

#endif //DMP_SUBSCRIBER_H
