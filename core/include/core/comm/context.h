#ifndef DMP_CONTEXT_H
#define DMP_CONTEXT_H

#include <zmq.hpp>

#include <memory>

namespace dmp
{
class Context
{
public:
  static std::shared_ptr<Context> getInstance()
  {
    static std::shared_ptr<Context> instance_;

    if (!instance_)
      instance_ = std::make_shared<Context>();
    return instance_;
  }

  Context()
      : zmq_context_(1)
  {
  }

  std::unique_ptr<zmq::socket_t> createPublisherSocket(std::string ip, int port = DEFAULT_PORT_NUMBER)
  {
    auto socket = std::make_unique<zmq::socket_t>(zmq_context_, ZMQ_PUB);
    socket->connect("tcp://" + ip + ":" + std::to_string(port));
    return socket;
  }

  std::unique_ptr<zmq::socket_t> createSubscriberSocket(std::string ip, int port = DEFAULT_PORT_NUMBER)
  {
    auto socket = std::make_unique<zmq::socket_t>(zmq_context_, ZMQ_SUB);
    socket->bind("tcp://" + ip + ":" + std::to_string(port));
    socket->setsockopt(ZMQ_SUBSCRIBE, nullptr, 0);
    return socket;
  }

private:
  const static int DEFAULT_PORT_NUMBER = 33133;

  zmq::context_t zmq_context_;
};
}

#endif //DMP_CONTEXT_H
