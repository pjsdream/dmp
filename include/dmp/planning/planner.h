#ifndef DMP_PLANNER_H
#define DMP_PLANNER_H

#include <memory>

namespace dmp
{
class Renderer;
class Planner
{
public:
  Planner();
  ~Planner();

  void setRenderer(const std::shared_ptr<Renderer>& renderer);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}

#endif //DMP_PLANNER_H
