#ifndef DMP_REQUEST_SUBSCRIBER_H
#define DMP_REQUEST_SUBSCRIBER_H

#include "request_mesh.h"

namespace dmp
{
// Subscriber specialization
template<>
std::unique_ptr<Request> Subscriber::receive()
{
  // Receive message through zmq
  zmq::message_t zmq_message;
  auto received = zmq_socket_->recv(&zmq_message, ZMQ_NOBLOCK);
  if (!received)
    return nullptr;

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
      auto req = std::make_unique<RequestMesh>();
      deserializer >> *req;
      return std::move(req);
    }

    case Request::Type::Clear:
      return nullptr;

    case Request::Type::CustomMesh:
      return nullptr;

    case Request::Type::CustomTexture:
      return nullptr;

    case Request::Type::Shape:
      return nullptr;

    case Request::Type::Frame:
      return nullptr;

    case Request::Type::Light:
      return nullptr;

    default:
      std::cerr << "received request of unknown type\n";
      return nullptr;
  }
}
}

#endif //DMP_REQUEST_SUBSCRIBER_H
