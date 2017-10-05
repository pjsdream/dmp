#include <dmp/comm/manager.h>
#include <dmp/comm/message_queue.h>
#include <dmp/comm/publisher.h>

namespace dmp
{
Manager::Manager() = default;

Manager::~Manager() = default;

template<typename T>
Publisher<T> Manager::createPublisher(const std::string& topic)
{
  if (queues_.find(topic) == queues_.end())
    queues_[topic] = std::make_shared<MessageQueue<T>>();

  return Publisher<T>(std::dynamic_pointer_cast<MessageQueue<T>>(queues_[topic]));
}
}
