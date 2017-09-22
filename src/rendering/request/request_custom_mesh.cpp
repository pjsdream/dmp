#include <dmp/rendering/request/request_custom_mesh.h>

namespace dmp
{
RequestCustomMesh::RequestCustomMesh()
    : Request(), has_global_color_(false)
{
}

bool RequestCustomMesh::hasGlobalColor()
{
  return has_global_color_;
}

const Eigen::Vector3f& RequestCustomMesh::getGlobalColor()
{
  return global_color_;
}

void RequestCustomMesh::setGlobalColor(const Eigen::Vector3f& global_color)
{
  has_global_color_ = true;
  global_color_ = global_color;
}

void RequestCustomMesh::createCube(Eigen::Vector3d size)
{
  // make it to halfsize
  size /= 2.;

  // down
  addVertex(-size(0), -size(1), -size(2), 0, 0, -1);
  addVertex(size(0), -size(1), -size(2), 0, 0, -1);
  addVertex(size(0), size(1), -size(2), 0, 0, -1);
  addVertex(-size(0), size(1), -size(2), 0, 0, -1);
  addFace(0, 2, 1);
  addFace(0, 3, 2);

  // up
  addVertex(-size(0), -size(1), size(2), 0, 0, 1);
  addVertex(size(0), -size(1), size(2), 0, 0, 1);
  addVertex(size(0), size(1), size(2), 0, 0, 1);
  addVertex(-size(0), size(1), size(2), 0, 0, 1);
  addFace(4, 5, 6);
  addFace(4, 6, 7);

  // left
  addVertex(-size(0), -size(1), -size(2), -1, 0, 0);
  addVertex(-size(0), size(1), -size(2), -1, 0, 0);
  addVertex(-size(0), size(1), size(2), -1, 0, 0);
  addVertex(-size(0), -size(1), size(2), -1, 0, 0);
  addFace(8, 10, 9);
  addFace(8, 11, 10);

  // right
  addVertex(size(0), -size(1), -size(2), 1, 0, 0);
  addVertex(size(0), size(1), -size(2), 1, 0, 0);
  addVertex(size(0), size(1), size(2), 1, 0, 0);
  addVertex(size(0), -size(1), size(2), 1, 0, 0);
  addFace(12, 13, 14);
  addFace(12, 14, 15);

  // front
  addVertex(-size(0), -size(1), -size(2), 0, -1, 0);
  addVertex(-size(0), -size(1), size(2), 0, -1, 0);
  addVertex(size(0), -size(1), size(2), 0, -1, 0);
  addVertex(size(0), -size(1), -size(2), 0, -1, 0);
  addFace(16, 18, 17);
  addFace(16, 19, 18);

  // back
  addVertex(-size(0), size(1), -size(2), 0, 1, 0);
  addVertex(-size(0), size(1), size(2), 0, 1, 0);
  addVertex(size(0), size(1), size(2), 0, 1, 0);
  addVertex(size(0), size(1), -size(2), 0, 1, 0);
  addFace(20, 21, 22);
  addFace(20, 22, 23);
}

void RequestCustomMesh::createCylinder(double r, double h, int n)
{
  // half size
  h /= 2.;

  for (int i = 0; i < n; i++)
  {
    auto t = (double) i / n * 2. * M_PI;
    auto c = cos(t);
    auto s = sin(t);

    addVertex(r * c, r * s, -h, c, s, 0.);
    addVertex(r * c, r * s, h, c, s, 0.);
  }

  for (int i = 0; i < n; i++)
  {
    double v0 = 2 * i;
    double v1 = 2 * i + 1;
    double v2 = (2 * i + 2) % (2 * n);
    double v3 = (2 * i + 3) % (2 * n);

    addFace(v0, v2, v1);
    addFace(v1, v2, v3);
  }

  addVertex(0., 0., -h, 0., 0., -1.);
  for (int i = 0; i < n; i++)
  {
    auto t = (double) i / n * 2. * M_PI;
    auto c = cos(t);
    auto s = sin(t);

    addVertex(r * c, r * s, -h, 0., 0., -1.);
  }

  for (int i = 0; i < n; i++)
  {
    double v0 = 2 * n + 1 + i;
    double v1 = 2 * n + 1 + ((i + 1) % n);

    addFace(2 * n, v1, v0);
  }

  addVertex(0., 0., h, 0., 0., 1.);
  for (int i = 0; i < n; i++)
  {
    auto t = (double) i / n * 2. * M_PI;
    auto c = cos(t);
    auto s = sin(t);

    addVertex(r * c, r * s, h, 0., 0., 1.);
  }

  for (int i = 0; i < n; i++)
  {
    double v0 = 3 * n + 2 + i;
    double v1 = 3 * n + 2 + ((i + 1) % n);

    addFace(3 * n + 1, v0, v1);
  }
}

void RequestCustomMesh::createSphere(double r, int subdivision)
{
  createSphere(r, Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(0., 1., 0.), Eigen::Vector3d(0., 0., 1.), subdivision);
  createSphere(r, Eigen::Vector3d(0., 1., 0.), Eigen::Vector3d(-1., 0., 0.), Eigen::Vector3d(0., 0., 1.), subdivision);
  createSphere(r, Eigen::Vector3d(-1., 0., 0.), Eigen::Vector3d(0., -1., 0.), Eigen::Vector3d(0., 0., 1.), subdivision);
  createSphere(r, Eigen::Vector3d(0., -1., 0.), Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(0., 0., 1.), subdivision);

  createSphere(r,
               Eigen::Vector3d(-1., 0., 0.),
               Eigen::Vector3d(0., -1., 0.),
               Eigen::Vector3d(0., 0., -1.),
               subdivision);
  createSphere(r, Eigen::Vector3d(0., -1., 0.), Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(0., 0., -1.), subdivision);
  createSphere(r, Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(0., 1., 0.), Eigen::Vector3d(0., 0., -1.), subdivision);
  createSphere(r, Eigen::Vector3d(0., 1., 0.), Eigen::Vector3d(-1., 0., 0.), Eigen::Vector3d(0., 0., -1.), subdivision);
}

void RequestCustomMesh::createSphere(double r,
                                     Eigen::Vector3d n0,
                                     Eigen::Vector3d n1,
                                     Eigen::Vector3d n2,
                                     int subdivision)
{
  if (subdivision == 0)
  {
    int f = vertex_buffer.size() / 3;
    addVertex(r * n0(0), r * n0(1), r * n0(2), n0(0), n0(1), n0(2));
    addVertex(r * n1(0), r * n1(1), r * n1(2), n1(0), n1(1), n1(2));
    addVertex(r * n2(0), r * n2(1), r * n2(2), n2(0), n2(1), n2(2));
    addFace(f, f + 1, f + 2);
    return;
  }

  Eigen::Vector3d n01 = (n0 + n1).normalized();
  Eigen::Vector3d n12 = (n1 + n2).normalized();
  Eigen::Vector3d n20 = (n2 + n0).normalized();

  createSphere(r, n0, n01, n20, subdivision - 1);
  createSphere(r, n1, n12, n01, subdivision - 1);
  createSphere(r, n2, n20, n12, subdivision - 1);
  createSphere(r, n01, n12, n20, subdivision - 1);
}

void RequestCustomMesh::addVertex(float x, float y, float z, float nx, float ny, float nz)
{
  vertex_buffer.push_back(x);
  vertex_buffer.push_back(y);
  vertex_buffer.push_back(z);
  normal_buffer.push_back(nx);
  normal_buffer.push_back(ny);
  normal_buffer.push_back(nz);
}

void RequestCustomMesh::addFace(int f0, int f1, int f2)
{
  face_buffer.push_back(f0);
  face_buffer.push_back(f1);
  face_buffer.push_back(f2);
}
}