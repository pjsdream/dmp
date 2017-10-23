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
  socket->connect(("tcp://" + ip + ":" + std::to_string(port)).c_str());

  return socket;
}

std::unique_ptr<zmq::socket_t> Context::createSubscriberSocket(std::string ip, int port)
{
  auto socket = std::make_unique<zmq::socket_t>(zmq_context_, ZMQ_SUB);
  socket->setsockopt(ZMQ_SUBSCRIBE, nullptr, 0);

  std::cout << "binding to " << ("tcp://" + ip + ":" + std::to_string(port)) << "\n";
  socket->bind(("tcp://" + ip + ":" + std::to_string(port)).c_str());

  std::cout << "binding to " << ("tcp://" + ip + ":" + std::to_string(port)) << " complete\n";

  return socket;
}
}
