#ifndef DMP_SUBSCRIBER_H
#define DMP_SUBSCRIBER_H

#include <type_traits>
#include <memory>

#include <dmp/comm/publisher.h>
#include <dmp/comm/message_queue.h>

namespace dmp
{
template<typename T>
class Subscriber
{
public:
  /*
  std::vector<std::unique_ptr<T>> popAll()
  {
    std::vector<std::unique_ptr<T>> result;
    for (auto& queue : queues_)
    {
      auto part = std::move(queue->popAll());
      for (auto& value : part)
        result.push_back(std::move(value));
    }
    return result;
  }

  void subscribeFrom(const Publisher<T>& publisher)
  {
    queues_.push_back(publisher.getQueue());
  }
   */

private:
  std::vector<std::shared_ptr<MessageQueue<T>>> queues_;
};
}

#endif //DMP_SUBSCRIBER_H
