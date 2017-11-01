#ifndef DMP_DISTANCE_QUERY_H
#define DMP_DISTANCE_QUERY_H

#include <Eigen/Dense>
#include <memory>

namespace dmp
{
class Shape;

class AABB;
class Cube;
class Cylinder;
class Sphere;

class DistanceQuery
{
public:
  DistanceQuery() = delete;
  explicit DistanceQuery(const Shape& object1, const Shape& object2) noexcept;

  void evaluate();

  double getDistance() const noexcept;
  double getPenetrationDepth() const noexcept;

private:
  const Shape& object1_;
  const Shape& object2_;

  template<typename T>
  void dispatchFirst(const T& object1);

  void evaluate(const AABB& object1, const AABB& object2);
  void evaluate(const AABB& object1, const Cube& object2);
  void evaluate(const AABB& object1, const Cylinder& object2);
  void evaluate(const AABB& object1, const Sphere& object2);

  void evaluate(const Cube& object1, const AABB& object2);
  void evaluate(const Cube& object1, const Cube& object2);
  void evaluate(const Cube& object1, const Cylinder& object2);
  void evaluate(const Cube& object1, const Sphere& object2);

  void evaluate(const Cylinder& object1, const AABB& object2);
  void evaluate(const Cylinder& object1, const Cube& object2);
  void evaluate(const Cylinder& object1, const Cylinder& object2);
  void evaluate(const Cylinder& object1, const Sphere& object2);

  void evaluate(const Sphere& object1, const AABB& object2);
  void evaluate(const Sphere& object1, const Cube& object2);
  void evaluate(const Sphere& object1, const Cylinder& object2);
  void evaluate(const Sphere& object1, const Sphere& object2);

  double distance_;
  double penetration_depth_;
};
}

#endif //DMP_DISTANCE_QUERY_H
