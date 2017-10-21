#ifndef DMP_MANAGER_H
#define DMP_MANAGER_H

#include "publisher.h"
#include "subscriber.h"

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
      publisher_queues_[topic] = std::make_shared<PublisherMessageQueue<T>>();

    return Publisher<T>(std::dynamic_pointer_cast<PublisherMessageQueue<T>>(publisher_queues_[topic]));
  }

  template<typename T>
  Subscriber<T> createSubscriber(const std::string& topic)
  {
    if (publisher_queues_.find(topic) == publisher_queues_.end())
      publisher_queues_[topic] = std::make_shared<PublisherMessageQueue<T>>();
    auto publisher_queue = std::dynamic_pointer_cast<PublisherMessageQueue<T>>(publisher_queues_[topic]);

    auto subscriber_queue = std::make_shared<SubscriberMessageQueue<T>>();
    Subscriber<T> subscriber{subscriber_queue};

    publisher_queue->addSubscriber(subscriber_queue);
    subscriber_queue->setPublisher(publisher_queue);

    return subscriber;
  }

private:
  std::unordered_map<std::string, std::shared_ptr<AbstractMessageQueue>> publisher_queues_;
};
}

#endif //DMP_MANAGER_H
