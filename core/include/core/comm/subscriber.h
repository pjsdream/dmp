#ifndef DMP_SUBSCRIBER_H
#define DMP_SUBSCRIBER_H

#include "context.h"
#include "zmq_deserializer.h"

#include <type_traits>
#include <memory>
#include <iostream>

namespace dmp
{
class Subscriber
{
public:
  Subscriber() = delete;

  explicit Subscriber(std::string ip)
      : zmq_socket_(Context::getInstance()->createSubscriberSocket(ip))
  {
  }

  // Returns true when it received message successfully
  template<typename T>
  std::unique_ptr<T> receive(T& v)
  {
    // Receive message through zmq
    zmq::message_t zmq_message;
    auto received = zmq_socket_->recv(&zmq_message, ZMQ_NOBLOCK);
    if (!received)
      return nullptr;

    // Deserialize
    auto result = std::make_unique<T>();
    ZmqDeserializer deserializer(zmq_message);
    deserializer >> *result;
    return result;
  }

private:
  std::unique_ptr<zmq::socket_t> zmq_socket_;
};
}

#endif //DMP_SUBSCRIBER_H
