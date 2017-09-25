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
  Publisher()
  {
    queue_ = std::make_shared<MessageQueue<T>>();
  }

  const std::shared_ptr<MessageQueue<T>>& getQueue() const
  {
    return queue_;
  }

  void publish(const T& value)
  {
    queue_->push(value);
  }

  void publish(T&& value)
  {
    queue_->push(std::move(value));
  }

  void publish(std::unique_ptr<T> value)
  {
    queue_->push(std::move(value));
  }

private:
  std::shared_ptr<MessageQueue<T>> queue_;
};
}

#endif //DMP_PUBLISHER_H
