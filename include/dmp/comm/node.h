#ifndef DMP_NODE_H
#define DMP_NODE_H

#include "manager.h"
#include "publisher.h"
#include "subscriber.h"

#include <thread>
#include <atomic>

namespace dmp
{
class Manager;

class Node
{
public:
  Node() = delete;
  Node(const std::shared_ptr<Manager>& manager, const std::string& name);
  virtual ~Node() = default;

  Node(const Node& rhs) = delete;
  Node& operator=(const Node& rhs) = delete;

  Node(Node&& rhs) = delete;
  Node& operator=(Node&& rhs) = delete;

  const std::string& getNodeName();

  void print(const char* format, ...);

  template<typename T>
  Publisher<T> createPublisher(const std::string& topic)
  {
    return manager_->createPublisher<T>(topic);
  }

  template<typename T>
  Subscriber<T> createSubscriber(const std::string& topic)
  {
    return manager_->createSubscriber<T>(topic);
  }

  // internal uses
  void runThread();
  void joinThread();

protected:
  virtual void run();

  bool stopRequested();

private:
  std::shared_ptr<Manager> manager_;

  std::string name_;

  std::thread thread_;
  std::atomic_bool stop_requested_;
};
}

#endif //DMP_NODE_H
