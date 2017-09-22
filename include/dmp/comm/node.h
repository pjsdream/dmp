#ifndef DMP_NODE_H
#define DMP_NODE_H

#include <thread>

#include <dmp/comm/message.h>

namespace dmp
{
class Node
{
public:
  Node() = delete;
  explicit Node(const std::string& name);
  virtual ~Node() = default;

  const std::string& getNodeName();

  void print(const char* format, ...);

  // internal uses
  void runThread();
  void joinThread();

protected:
  virtual void run();

private:
  std::string name_;

  std::thread thread_;
};
}

#endif //DMP_NODE_H
