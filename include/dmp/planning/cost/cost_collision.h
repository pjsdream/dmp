#ifndef DMP_COST_COLLISION_H
#define DMP_COST_COLLISION_H

#include "cost.h"

namespace dmp
{
class CostCollision : public Cost
{
public:
  explicit CostCollision(double weight);

private:
};
}

#endif //DMP_COST_COLLISION_H
