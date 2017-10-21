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
  // half extents
  Eigen::VectorXd sizes(6);
  sizes.block(0, 0, 3, 1) = object1.getSize() / 2.;
  sizes.block(3, 0, 3, 1) = object2.getSize() / 2.;

  // find normal candidates
  Eigen::Matrix<double, 3, 15> normals;

  normals.block(0, 0, 3, 3) = object1.getTransform().linear();
  normals.block(0, 3, 3, 3) = object2.getTransform().linear();

  Eigen::Matrix3d outer;
  outer(0, 0) = 0.;
  outer(1, 1) = 0.;
  outer(2, 2) = 0.;
  for (int i = 0; i < 3; i++)
  {
    outer(1, 0) = normals(2, i);
    outer(2, 0) = -normals(1, i);
    outer(0, 1) = -normals(2, i);
    outer(2, 1) = normals(0, i);
    outer(0, 2) = normals(1, i);
    outer(1, 2) = -normals(0, i);

    normals.block(0, (i + 2) * 3, 3, 3).noalias() = outer * normals.block(0, 3, 3, 3);
  }

  // endpoint projection
  double min_dist = 1e10;
  bool is_outside = false;

  for (int i = 0; i < 15; i++)
  {
    const Eigen::Vector3d& v = normals.col(i);
    const double norm = v.norm();
    if (norm > 1e-4)
    {
      double proj_min_endpoints = v.dot(object1.getTransform().translation() - object2.getTransform().translation());
      double proj_max_endpoints = proj_min_endpoints;

      for (int j = 0; j < 6; j++)
      {
        const double d = std::abs(v.dot(normals.col(j)));
        proj_min_endpoints -= d * sizes(j);
        proj_max_endpoints += d * sizes(j);
      }

      // outside determination
      if (proj_min_endpoints * proj_max_endpoints >= 0)
      {
        is_outside = true;
        penetration_depth_ = 0.;
        // TODO: compute distance
      }

      proj_min_endpoints /= norm;
      proj_max_endpoints /= norm;

      if (min_dist > -proj_min_endpoints)
        min_dist = -proj_min_endpoints;
      if (min_dist > proj_max_endpoints)
        min_dist = proj_max_endpoints;
    }
  }

  if (!is_outside)
  {
    penetration_depth_ = min_dist;
    // TODO: compute distance
  }
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
