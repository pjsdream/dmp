#ifndef DMP_MESSAGE_QUEUE_H
#define DMP_MESSAGE_QUEUE_H

#include <type_traits>
#include <mutex>

#include <dmp/comm/message.h>

namespace dmp
{
class AbstractMessageQueue
{
public:
  virtual ~AbstractMessageQueue() = default;
};

template<typename T, typename = typename std::enable_if_t<std::is_base_of<Message, T>::value>>
class MessageQueue : public AbstractMessageQueue
{
public:
  ~MessageQueue() = default;

  void push(const T& value)
  {
    std::lock_guard<std::mutex> lock{mutex_};
    queue_.push_back(std::make_unique<T>(value));
  }

  void push(T&& value)
  {
    std::lock_guard<std::mutex> lock{mutex_};
    queue_.push_back(std::make_unique<T>(std::move(value)));
  }

  void push(std::unique_ptr<T> value)
  {
    std::lock_guard<std::mutex> lock{mutex_};
    queue_.push_back(std::move(value));
  }

  std::vector<std::unique_ptr<T>> popAll()
  {
    std::vector<std::unique_ptr<T>> result;

    {
      std::lock_guard<std::mutex> lock{mutex_};
      queue_.swap(result);
    }

    return result;
  }

private:
  std::mutex mutex_;
  std::vector<std::unique_ptr<T>> queue_;
};
}

#endif //DMP_MESSAGE_QUEUE_H
