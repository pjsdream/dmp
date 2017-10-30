#ifndef DMP_REQUEST_SUBSCRIBER_H
#define DMP_REQUEST_SUBSCRIBER_H

#include "request_mesh.h"

#include <core/comm/subscriber.h>

namespace dmp
{
// Subscriber specialization
template<>
bool Subscriber::receive(RequestMesh& result)
{
  // Receive message through zmq
  zmq::message_t zmq_message;
  auto received = zmq_socket_->recv(&zmq_message, ZMQ_NOBLOCK);
  if (!received)
    return false;

  std::cout << "message size: " << zmq_message.size() <<"\n";
  for (int i=0; i<zmq_message.size(); i++)
    std::cout << static_cast<int>(*(zmq_message.data<char>() + i)) << ' ';
  std::cout << "\n";

  // Deserialize
  ZmqDeserializer type_discreminator(zmq_message);
  Request::Type type;
  type_discreminator >> type;

  ZmqDeserializer deserializer(zmq_message);
  switch (type)
  {
    case Request::Type::Mesh:
    {
      deserializer >> result;
    }

    case Request::Type::Clear:
      return true;

    case Request::Type::CustomMesh:
      return true;

    case Request::Type::CustomTexture:
      return true;

    case Request::Type::Shape:
      return true;

    case Request::Type::Frame:
      return true;

    case Request::Type::Light:
      return true;

    default:
      std::cerr << "received request of unknown type\n";
      return false;
  }
}
}

#endif //DMP_REQUEST_SUBSCRIBER_H
