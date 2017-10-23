#ifndef DMP_CONTEXT_H
#define DMP_CONTEXT_H

#include <zmq.hpp>

#include <memory>

namespace dmp
{
class Context
{
public:
  static std::shared_ptr<Context> getInstance();

  Context();

  std::unique_ptr<zmq::socket_t> createPublisherSocket(std::string ip, int port = DEFAULT_PORT_NUMBER);
  std::unique_ptr<zmq::socket_t> createSubscriberSocket(std::string ip, int port = DEFAULT_PORT_NUMBER);

private:
  const static int DEFAULT_PORT_NUMBER = 33133;

  static std::shared_ptr<Context> instance_;

  zmq::context_t zmq_context_;
};
}

#endif //DMP_CONTEXT_H
