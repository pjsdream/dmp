#ifndef DMP_VECTOR_EIGEN_H
#define DMP_VECTOR_EIGEN_H

#include <Eigen/Dense>
#include <map>

namespace dmp
{
template<typename T>
using VectorEigen = std::vector<T, Eigen::aligned_allocator<T>>;

template<typename T1, typename T2>
using MapEigen = std::map<T1, T2, std::less<T1>, Eigen::aligned_allocator<std::pair<const T1, T2>>>;
}

#endif //DMP_VECTOR_EIGEN_H
