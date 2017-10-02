#ifndef DMP_COST_SMOOTHNESS_H
#define DMP_COST_SMOOTHNESS_H

#include "cost.h"

namespace dmp
{
class CostSmoothness : public Cost
{
public:
  explicit CostSmoothness(double weight);
  ~CostSmoothness() override = default;

  std::tuple<double, Eigen::VectorXd, Eigen::VectorXd> computeCost(const RobotConfiguration& configuration) override;

private:
};
}

#endif //DMP_COST_SMOOTHNESS_H
