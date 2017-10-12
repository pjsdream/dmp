#ifndef DMP_SUBSCRIBER_H
#define DMP_SUBSCRIBER_H

#include <type_traits>
#include <memory>

#include <dmp/comm/message_queue.h>

namespace dmp
{
template<typename T>
class Subscriber
{
public:
  Subscriber() = default;

  explicit Subscriber(const std::shared_ptr<SubscriberMessageQueue<T>>& queue) noexcept
      : queue_(queue)
  {
  }

  std::shared_ptr<T> pop()
  {
    if (queue_)
      return queue_->pop();
    return nullptr;
  }

  std::vector<std::shared_ptr<T>> popAll()
  {
    if (queue_)
      return queue_->popAll();
    return std::vector<std::shared_ptr<T>>();
  }

private:
  std::shared_ptr<SubscriberMessageQueue<T>> queue_;
};
}

#endif //DMP_SUBSCRIBER_H
