#ifndef DMP_OBJECTIVE_H
#define DMP_OBJECTIVE_H

#include <Eigen/Dense>

#include <type_traits>

namespace dmp
{
class RobotConfiguration;

class Objective
{
public:
  virtual ~Objective() = default;

  template<typename T, typename = typename std::enable_if_t<std::is_base_of<Objective, T>::value>>
  T& as()
  {
    return static_cast<T>(*this);
  };

  virtual std::tuple<double, Eigen::VectorXd, Eigen::VectorXd> computeCost(const RobotConfiguration& configuration);

private:
};
}

#endif //DMP_OBJECTIVE_H
