#ifndef DMP_CONTEXT_H
#define DMP_CONTEXT_H

#include <zmq.hpp>

namespace dmp
{
class Context
{
public:
  Context();

  zmq::socket_t createPublisherSocket(int port = DEFAULT_PORT_NUMBER);
  zmq::socket_t createSubscriberSocket(std::string ip, int port = DEFAULT_PORT_NUMBER);

private:
  const static int DEFAULT_PORT_NUMBER = 33133;

  zmq::context_t zmq_context_;
};
}

#endif //DMP_CONTEXT_H
