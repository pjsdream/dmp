#ifndef DMP_SUBSCRIBER_H
#define DMP_SUBSCRIBER_H

#include "context.h"
#include "deserializer.h"

#include <type_traits>
#include <memory>
#include <iostream>

namespace dmp
{
template<typename T>
class Subscriber
{
public:
  Subscriber() = delete;

  Subscriber(Context& context, std::string ip, std::string topic)
      : zmq_socket_(context.createSubscriberSocket(ip, topic))
  {
  }

  // Returns true when it received message successfully
  bool receive(T& result)
  {
    // Receive message through zmq
    zmq::message_t zmq_message;
    auto received = zmq_socket_.recv(&zmq_message, ZMQ_NOBLOCK);
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
  zmq::socket_t zmq_socket_;
};
}

#endif //DMP_SUBSCRIBER_H
