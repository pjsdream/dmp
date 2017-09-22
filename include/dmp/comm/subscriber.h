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
    if (queue_)
      return queue_->popAll();
    return std::vector<std::unique_ptr<T>>();
  }

  // for internal use only
  void conntectToQueue(const std::shared_ptr<MessageQueue<T>>& queue)
  {
    queue_ = queue;
  }

private:
  std::shared_ptr<MessageQueue<T>> queue_;
};
}

#endif //DMP_SUBSCRIBER_H
