#include <core/comm/context.h>

#include <iostream>

namespace dmp
{
Context::Context()
    : zmq_context_(1)
{
}

zmq::socket_t Context::createPublisherSocket(int port)
{
  zmq::socket_t socket(zmq_context_, ZMQ_PUB);
  socket.bind(("tcp://*:" + std::to_string(port)).c_str());

  std::cout << "binding to " << ("tcp://*:" + std::to_string(port)) << "\n";

  return socket;
}

zmq::socket_t Context::createSubscriberSocket(std::string ip, int port)
{
  zmq::socket_t socket(zmq_context_, ZMQ_SUB);
  socket.connect(("tcp://" + ip + ":" + std::to_string(port)).c_str());
  socket.setsockopt(ZMQ_SUBSCRIBE, nullptr, 0);

  std::cout << "connecting to " << ("tcp://" + ip + ":" + std::to_string(port)) << "\n";

  return socket;
}
}
