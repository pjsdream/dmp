#ifndef DMP_PUBLISHER_H
#define DMP_PUBLISHER_H

#include <type_traits>

namespace dmp
{
template<typename T>
class MessageQueue;

template<typename T>
class Publisher
{
public:
  Publisher() = default;

  Publisher(const std::shared_ptr<MessageQueue<T>>& queue)
      : queue_(queue)
  {
  }

  void publish(const T& value)
  {
    if (queue_)
      queue_->push(value);
  }

  void publish(T&& value)
  {
    if (queue_)
      queue_->push(std::move(value));
  }

  void publish(std::unique_ptr<T> value)
  {
    if (queue_)
      queue_->push(std::move(value));
  }

private:
  std::shared_ptr<MessageQueue<T>> queue_;
};
}

#endif //DMP_PUBLISHER_H
