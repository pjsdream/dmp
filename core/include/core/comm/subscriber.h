#ifndef DMP_SUBSCRIBER_H
#define DMP_SUBSCRIBER_H

#include "context.h"
#include "zmq_serializer.h"
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

  Subscriber(std::string topic, std::string ip)
      : zmq_socket_(Context::getInstance()->createSubscriberSocket(ip))
  {
    // Set up filter
    ZmqSerializerSizeEvaluator size_evaluator;
    size_evaluator << topic;

    ZmqSerializer serializer(size_evaluator.size());
    serializer << topic;

    const auto& message = serializer.getMessage();
    printf("%d %s\n", message.size(), message.data<char>() + 8);

    zmq_socket_->setsockopt(ZMQ_SUBSCRIBE, message.data(), message.size());
  }

  // Returns true when it received message successfully
  template<typename T>
  bool receive(T& v)
  {
    // Receive message through zmq
    zmq::message_t zmq_message;
    auto received = zmq_socket_->recv(&zmq_message, ZMQ_NOBLOCK);
    if (!received)
      return false;

    // Deserialize
    std::string topic;
    ZmqDeserializer deserializer(zmq_message);
    deserializer >> topic >> v;

    return true;
  }

private:
  std::unique_ptr<zmq::socket_t> zmq_socket_;
};
}

#endif //DMP_SUBSCRIBER_H
