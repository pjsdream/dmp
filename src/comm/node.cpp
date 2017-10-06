#include <dmp/comm/node.h>
#include <cstdarg>

namespace dmp
{
Node::Node(const std::shared_ptr<Manager>& manager, const std::string& name)
    : manager_(manager), name_(name), stop_requested_(false)
{
}

const std::string& Node::getNodeName()
{
  return name_;
}

void Node::print(const char* format, ...)
{
  printf("[%s] ", name_.c_str());

  va_list argptr;
  va_start(argptr, format);
  vprintf(format, argptr);
  va_end(argptr);
}

void Node::runThread()
{
  thread_ = std::thread([node = this]()
                        {
                          node->run();
                        });
}

void Node::joinThread()
{
  thread_.join();
}

void Node::run()
{
  printf("Node::run() should not be running\n");
}

bool Node::stopRequested()
{
  return stop_requested_;
}
}
