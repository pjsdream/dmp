#ifndef DMP_MESSAGE_QUEUE_H
#define DMP_MESSAGE_QUEUE_H

#include <type_traits>
#include <mutex>
#include <memory>
#include <vector>

#include <queue>

namespace dmp
{
template<typename T>
class SubscriberMessageQueue;

class AbstractMessageQueue
{
public:
  AbstractMessageQueue() = default;
  virtual ~AbstractMessageQueue() = default;

  AbstractMessageQueue(const AbstractMessageQueue& rhs) = delete;
  AbstractMessageQueue& operator=(const AbstractMessageQueue& rhs) = delete;

  AbstractMessageQueue(AbstractMessageQueue&& rhs) = delete;
  AbstractMessageQueue& operator=(AbstractMessageQueue&& rhs) = delete;
};

template<typename T>
class MessageQueue : public AbstractMessageQueue
{
public:
  MessageQueue() = default;
  ~MessageQueue() override = default;

  MessageQueue(const MessageQueue& rhs) = delete;
  MessageQueue& operator=(const MessageQueue& rhs) = delete;

  MessageQueue(MessageQueue&& rhs) = delete;
  MessageQueue& operator=(MessageQueue&& rhs) = delete;

  void push(const T& value)
  {
    std::lock_guard<std::mutex> lock{mutex_};
    queue_.push(std::make_shared<T>(value));
  }

  void push(T&& value)
  {
    std::lock_guard<std::mutex> lock{mutex_};
    queue_.push(std::make_shared<T>(std::move(value)));
  }

  void push(std::unique_ptr<T> value)
  {
    std::lock_guard<std::mutex> lock{mutex_};
    queue_.push(std::shared_ptr<T>(std::move(value)));
  }

  virtual std::shared_ptr<T> pop()
  {
    std::lock_guard<std::mutex> lock{mutex_};

    auto front = queue_.front();
    queue_.pop();
    return front;
  }

protected:
  std::mutex mutex_;
  std::queue<std::shared_ptr<T>> queue_{};
};

template<typename T>
class PublisherMessageQueue : public MessageQueue<T>
{
public:
  PublisherMessageQueue() = default;
  ~PublisherMessageQueue() override = default;

  PublisherMessageQueue(const PublisherMessageQueue& rhs) = delete;
  PublisherMessageQueue& operator=(const PublisherMessageQueue& rhs) = delete;

  PublisherMessageQueue(PublisherMessageQueue&& rhs) = delete;
  PublisherMessageQueue& operator=(PublisherMessageQueue&& rhs) = delete;

  void addSubscriber(const std::shared_ptr<SubscriberMessageQueue<T>>& subscriber)
  {
    subscribers_.push_back(subscriber);
  }

  void broadcast();

private:
  std::vector<std::shared_ptr<SubscriberMessageQueue<T>>> subscribers_{};
};

template<typename T>
class SubscriberMessageQueue : public MessageQueue<T>
{
public:
  SubscriberMessageQueue() = default;
  ~SubscriberMessageQueue() override = default;

  SubscriberMessageQueue(const SubscriberMessageQueue& rhs) = delete;
  SubscriberMessageQueue& operator=(const SubscriberMessageQueue& rhs) = delete;

  SubscriberMessageQueue(SubscriberMessageQueue&& rhs) = delete;
  SubscriberMessageQueue& operator=(SubscriberMessageQueue&& rhs) = delete;

  void setPublisher(const std::shared_ptr<SubscriberMessageQueue<T>>& publisher)
  {
    publisher_ = publisher;
  }

  std::shared_ptr<T> pop() override
  {
    std::lock_guard<std::mutex> lock{this->mutex_};

    // Obtaining shared ptr to publisher
    if (auto publisher = publisher_.lock())
    {
      // Request broadcast to the publisher queue
      publisher->broadcast();

      // Behaves like pop() of the basis function, but the mutex is already locked.
      auto front = this->queue_.front();
      this->queue_.pop();
      return front;
    }

    return nullptr;
  }

private:
  std::weak_ptr<PublisherMessageQueue<T>> publisher_;
};

//
// Implementations
//
template<typename T>
void PublisherMessageQueue<T>::broadcast()
{
  std::lock_guard<std::mutex> lock{this->mutex_};

  // Broadcast pending messages to all connected subscribers
  while (!this->queue_.empty())
  {
    auto message = this->queue_.front();
    this->queue_.pop();

    for (const auto& subscriber : subscribers_)
      subscriber->push(message);
  }
}
}

#endif //DMP_MESSAGE_QUEUE_H
