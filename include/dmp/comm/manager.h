#ifndef DMP_MANAGER_H
#define DMP_MANAGER_H

#include "publisher.h"
#include "subscriber.h"
#include "message_queue.h"

#include <unordered_map>
#include <memory>

namespace dmp
{
class Manager
{
public:
  Manager();
  ~Manager();

  template<typename T>
  Publisher<T> createPublisher(const std::string& topic)
  {
    if (publisher_queues_.find(topic) == publisher_queues_.end())
      publisher_queues_[topic] = std::make_shared<MessageQueue<T>>();

    return Publisher<T>(std::dynamic_pointer_cast<PublisherMessageQueue<T>>(publisher_queues_[topic]));
  }

  template<typename T>
  Subscriber<T> createSubscriber(const std::string& topic)
  {
    // TODO
    return Subscriber<T>();
  }

private:
  std::unordered_map<std::string, std::shared_ptr<AbstractMessageQueue>> publisher_queues_;
};
}

#endif //DMP_MANAGER_H
