#ifndef DMP_SUBSCRIBER_H
#define DMP_SUBSCRIBER_H

#include "context.h"
#include "serializer.h"
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

  Subscriber(std::string topic, std::string ip)
      : deserializer_(nullptr), zmq_socket_(Context::getInstance()->createSubscriberSocket(ip))
  {
    // Set up filter
    std::vector<char> buffer;
    Serializer serializer(buffer);
    serializer << topic;

    zmq_socket_->setsockopt(ZMQ_SUBSCRIBE, buffer.data(), buffer.size());
  }

  // Returns true when it received message successfully
  bool eof()
  {
    if (message_received_ && deserializer_.pointer() < zmq_message_.data<char>() + zmq_message_.size())
      return false;
    message_received_ = false;

    // Receive message through zmq
    auto received = zmq_socket_->recv(&zmq_message_, ZMQ_NOBLOCK);
    if (!received)
      return true;
    message_received_ = true;

    // Consume topic
    std::string topic;
    deserializer_ = Deserializer(zmq_message_.data<char>());
    deserializer_ >> topic;

    return false;
  }

  template<typename T>
  Subscriber& operator>>(T& v)
  {
    deserializer_ >> v;
    return *this;
  }

  Subscriber& operator<<(int size)
  {
    deserializer_ << size;
    return *this;
  }

private:
  bool message_received_ = false;
  zmq::message_t zmq_message_;
  std::unique_ptr<zmq::socket_t> zmq_socket_;
  Deserializer deserializer_;
};
}

#endif //DMP_SUBSCRIBER_H
