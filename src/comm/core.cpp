#include <dmp/comm/core.h>
#include <dmp/comm/node.h>

namespace dmp
{
std::vector<std::shared_ptr<Node>> Core::nodes_;
std::unordered_map<std::string, int> Core::node_names_;
std::vector<std::shared_ptr<AbstractMessageQueue>> Core::queues_;

void Core::init(int argc, char** argv)
{
}

void Core::addNode(const std::shared_ptr<Node>& node)
{
  nodes_.push_back(node);
}

void Core::run()
{
  for (const auto& node : nodes_)
    node->runThread();

  // wait until all threads are done running
  for (const auto& node : nodes_)
    node->joinThread();
}

std::string Core::assignNodeName(const std::string& name)
{
  auto it = node_names_.find(name);

  if (it == node_names_.cend())
  {
    node_names_[name] = 1;
    return name;
  }

  // node name already exists
  const auto num = it->second;
  node_names_[name]++;
  return name + "_" + std::to_string(num);
}
}
