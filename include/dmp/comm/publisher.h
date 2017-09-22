#ifndef DMP_PUBLISHER_H
#define DMP_PUBLISHER_H

#include <type_traits>

#include <dmp/comm/message.h>
#include <dmp/comm/message_queue.h>

namespace dmp
{
template<typename T, typename = typename std::enable_if_t<std::is_base_of<Message, T>::value>>
class Publisher
{
public:
  void publish(const T& value)
  {
    if (queue_)
      queue_->push(value);
  }

  void publish(std::unique_ptr<T> value)
  {
    if (queue_)
      queue_->push(std::move(value));
  }

  // for internal use only
  void conntectToQueue(const std::shared_ptr<MessageQueue<T>>& queue)
  {
    queue_ = queue;
  }

private:
  // Core class should manage the queue so that the reference won't dangle
  std::shared_ptr<MessageQueue<T>> queue_;
};
}

#endif //DMP_PUBLISHER_H
