#ifndef DMP_CORE_H
#define DMP_CORE_H

#include <vector>
#include <memory>
#include <unordered_map>

#include <dmp/comm/message.h>
#include <dmp/comm/publisher.h>
#include <dmp/comm/subscriber.h>

namespace dmp
{
class Node;
class Core
{
public:
  static void init(int argc, char** argv);
  static void addNode(const std::shared_ptr<Node>& node);
  static void run();

  template<typename T, typename = typename std::enable_if_t<std::is_base_of<Message, T>::value>>
  static void connect(Publisher<T>& publisher, Subscriber<T>& subscriber)
  {
    // create a queue object
    auto queue = std::make_shared<MessageQueue<T>>();

    publisher.conntectToQueue(queue);
    subscriber.conntectToQueue(queue);

    // store in static
    queues_.push_back(queue);
  }

  // functions for internal uses
  static std::string assignNodeName(const std::string& name);

private:
  static std::vector<std::shared_ptr<Node>> nodes_;
  static std::unordered_map<std::string, int> node_names_;
  static std::vector<std::shared_ptr<AbstractMessageQueue>> queues_;
};
}

#endif //DMP_CORE_H
