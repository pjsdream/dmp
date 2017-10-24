#include <core/comm/context.h>

#include <iostream>

namespace dmp
{
std::shared_ptr<Context> Context::instance_;

std::shared_ptr<Context> Context::getInstance()
{
  if (!instance_)
    instance_ = std::make_shared<Context>();
  return instance_;
}

Context::Context()
    : zmq_context_(1)
{
}

std::unique_ptr<zmq::socket_t> Context::createPublisherSocket(std::string ip, int port)
{
  auto socket = std::make_unique<zmq::socket_t>(zmq_context_, ZMQ_PUB);

  std::cout << "connecting to " << ("tcp://" + ip + ":" + std::to_string(port)) << "\n";
  socket->connect("tcp://" + ip + ":" + std::to_string(port));

  return socket;
}

std::unique_ptr<zmq::socket_t> Context::createSubscriberSocket(std::string ip, int port)
{
  auto socket = std::make_unique<zmq::socket_t>(zmq_context_, ZMQ_SUB);

  std::cout << "binding to " << ("tcp://" + ip + ":" + std::to_string(port)) << "\n";
  socket->bind("tcp://" + ip + ":" + std::to_string(port));

  socket->setsockopt(ZMQ_SUBSCRIBE, nullptr, 0);

  return socket;
}
}
