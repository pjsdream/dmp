#ifndef DMP_SUBSCRIBER_H
#define DMP_SUBSCRIBER_H

#include <type_traits>
#include <memory>

#include <dmp/comm/message.h>
#include <dmp/comm/message_queue.h>

namespace dmp
{
template<typename T, typename = typename std::enable_if_t<std::is_base_of<Message, T>::value>>
class Subscriber
{
public:
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

  // for internal use only
  void conntectToQueue(const std::shared_ptr<MessageQueue<T>>& queue)
  {
    queues_.push_back(queue);
  }

private:
  std::vector<std::shared_ptr<MessageQueue<T>>> queues_;
};
}

#endif //DMP_SUBSCRIBER_H
