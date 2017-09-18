#include <dmp/shape/distance_query.h>
#include <dmp/shape/shape.h>
#include <dmp/shape/aabb.h>
#include <dmp/shape/cube.h>
#include <dmp/shape/cylinder.h>
#include <dmp/shape/sphere.h>

namespace dmp
{
DistanceQuery::DistanceQuery(const Shape& object1, const Shape& object2) noexcept
    : object1_(object1), object2_(object2)
{
}

void DistanceQuery::evaluate()
{
  switch (object1_.getType())
  {
    case Shape::Type::AABB:
      dispatchFirst(object1_.as<AABB>());
      break;
    case Shape::Type::Cube:
      dispatchFirst(object1_.as<Cube>());
      break;
    case Shape::Type::Cylinder:
      dispatchFirst(object1_.as<Cylinder>());
      break;
    case Shape::Type::Sphere:
      dispatchFirst(object1_.as<Sphere>());
      break;
  }
}

template<typename T>
void DistanceQuery::dispatchFirst(const T& object1)
{
  switch (object2_.getType())
  {
    case Shape::Type::AABB:
      evaluate(object1, object2_.as<AABB>());
      break;
    case Shape::Type::Cube:
      evaluate(object1, object2_.as<Cube>());
      break;
    case Shape::Type::Cylinder:
      evaluate(object1, object2_.as<Cylinder>());
      break;
    case Shape::Type::Sphere:
      evaluate(object1, object2_.as<Sphere>());
      break;
  }
}

double DistanceQuery::getDistance() const noexcept
{
  return distance_;
}

double DistanceQuery::getPenetrationDepth() const noexcept
{
  return penetration_depth_;
}

//
// AABB
//
void DistanceQuery::evaluate(const AABB& object1, const AABB& object2)
{
  // TODO
}

void DistanceQuery::evaluate(const AABB& object1, const Cube& object2)
{
  // TODO
}

void DistanceQuery::evaluate(const AABB& object1, const Cylinder& object2)
{
  // TODO
}

void DistanceQuery::evaluate(const AABB& object1, const Sphere& object2)
{
  // TODO
}


//
// Cube (OBB)
//
void DistanceQuery::evaluate(const Cube& object1, const AABB& object2)
{
  evaluate(object2, object1);
}

void DistanceQuery::evaluate(const Cube& object1, const Cube& object2)
{
  // TODO
}

void DistanceQuery::evaluate(const Cube& object1, const Cylinder& object2)
{
  // TODO
}

void DistanceQuery::evaluate(const Cube& object1, const Sphere& object2)
{
  // TODO
}


//
// Cylinder
//
void DistanceQuery::evaluate(const Cylinder& object1, const AABB& object2)
{
  evaluate(object2, object1);
}

void DistanceQuery::evaluate(const Cylinder& object1, const Cube& object2)
{
  evaluate(object2, object1);
}

void DistanceQuery::evaluate(const Cylinder& object1, const Cylinder& object2)
{
  // TODO
}

void DistanceQuery::evaluate(const Cylinder& object1, const Sphere& object2)
{
  // TODO
}


//
// Sphere
//
void DistanceQuery::evaluate(const Sphere& object1, const AABB& object2)
{
  evaluate(object2, object1);
}

void DistanceQuery::evaluate(const Sphere& object1, const Cube& object2)
{
  evaluate(object2, object1);
}

void DistanceQuery::evaluate(const Sphere& object1, const Cylinder& object2)
{
  evaluate(object2, object1);
}

void DistanceQuery::evaluate(const Sphere& object1, const Sphere& object2)
{
  auto d_square = (object1.getPosition() - object2.getPosition()).squaredNorm();
  auto r1 = object1.getRadius();
  auto r2 = object2.getRadius();
  auto r_sum = r1 + r2;

  if (r_sum * r_sum < d_square)
  {
    distance_ = std::sqrt(d_square) - r_sum;
    penetration_depth_ = 0.;
  }
  else
  {
    distance_ = 0.;
    penetration_depth_ = std::sqrt(d_square) - r_sum;
  }
}
}
