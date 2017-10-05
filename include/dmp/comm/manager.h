#ifndef DMP_MANAGER_H
#define DMP_MANAGER_H

#include <unordered_map>
#include <memory>

namespace dmp
{
template<typename T>
class Publisher;

class AbstractMessageQueue;

class Manager
{
public:
  Manager();
  ~Manager();

  template<typename T>
  Publisher<T> createPublisher(const std::string& topic);

private:
  std::unordered_map<std::string, std::shared_ptr<AbstractMessageQueue>> queues_;
};
}

#endif //DMP_MANAGER_H
