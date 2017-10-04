#ifndef DMP_MATRIX_H
#define DMP_MATRIX_H

#include <type_traits>
#include <cstring>
#include <array>

// Uncomment if
#define DMP_MATRIX_DEBUG

namespace dmp
{
template<typename Scalar, int n, int m, typename = typename std::enable_if_t<std::is_floating_point<Scalar>::value>>
class Matrix
{
public:
  Matrix() noexcept
  {
    printDebug("Constructor\n");
  }

  Matrix(const Matrix& rhs)
  {
    printDebug("Copy constructor\n");
    data_ = rhs.data_;
  }

  Matrix& operator=(const Matrix& rhs)
  {
    printDebug("Copy operator\n");
    data_ = rhs.data_;
  }

  ~Matrix()
  {
    printDebug("Destructor\n");
  }

  // Move is the same as copy
  Matrix(Matrix&& rhs)
  {
    printDebug("Move constructor\n");
    data_ = std::move(rhs.data_);
  }

  // Move is the same as copy
  Matrix& operator=(Matrix&& rhs)
  {
    printDebug("Move operator\n");
    data_ = std::move(rhs.data_);
  }

  //
  // Static initializer
  //
  static constexpr Matrix Zero()
  {
    Matrix result;
    std::memset(result.data_.data(), 0, numBytes());
    return result;
  }

  template<typename = typename std::enable_if_t<n == m>>
  static Matrix Identity()
  {
    Matrix result = Zero();
    for (int i=0; i<n; i++)
      result(i, i) = static_cast<Scalar>(1.);
    return result;
  }

  //
  // Reference
  //
  Scalar& operator()(int i, int j) noexcept
  {
    return data_[j * n + i];
  }

  const Scalar& operator()(int i, int j) const noexcept
  {
    return data_[j * n + i];
  }

  //
  // Matrix multiplication
  //
  template<int k>
  Matrix<Scalar, n, k> operator*(const Matrix<Scalar, m, k>& rhs)
  {
    Matrix<Scalar, n, k> result;
    for (int i = 0; i < n; i++)
    {
      for (int j = 0; j < k; j++)
      {
        result(i, j) = static_cast<Scalar>(0.);

        for (int l = 0; l < m; l++)
          result(i, j) += (*this)(i, l) * rhs(l, j);
      }
    }

    return result;
  }

  template<int k>
  Matrix<Scalar, n, k> operator*(Matrix<Scalar, m, k>&& rhs)
  {
    Matrix<Scalar, n, k> result;
    for (int i = 0; i < n; i++)
    {
      for (int j = 0; j < k; j++)
      {
        result(i, j) = static_cast<Scalar>(0.);

        for (int l = 0; l < m; l++)
          result(i, j) += (*this)(i, l) * rhs(l, j);
      }
    }

    return result;
  }

  template<typename = typename std::enable_if_t<n == m>>
  Matrix& operator*=(const Matrix& rhs)
  {
    Matrix result;
    for (int i = 0; i < n; i++)
    {
      for (int j = 0; j < m; j++)
      {
        result(i, j) = static_cast<Scalar>(0.);

        for (int l = 0; l < m; l++)
          result(i, j) += (*this)(i, l) * rhs(l, j);
      }
    }

    *this = result;
    return *this;
  }

  template<typename = typename std::enable_if_t<n == m>>
  Matrix& operator*=(Matrix&& rhs)
  {
    Matrix result;
    for (int i = 0; i < n; i++)
    {
      for (int j = 0; j < m; j++)
      {
        result(i, j) = static_cast<Scalar>(0.);

        for (int l = 0; l < m; l++)
          result(i, j) += (*this)(i, l) * rhs(l, j);
      }
    }

    *this = result;
    return *this;
  }

  //
  // Scalar multiplication
  //
  Matrix operator*(Scalar rhs)
  {
    Matrix result{*this};
    for (int i = 0; i < numElements(); i++)
      result.data_[i] *= rhs;
    return result;
  }

private:
  static constexpr auto numElements() -> decltype(n * m)
  {
    return n * m;
  }

  static constexpr auto numBytes() -> decltype(sizeof(Scalar))
  {
    return numElements() * sizeof(Scalar);
  }

  //
  // Printing function for debugging
  //
  void printDebug(const std::string& s)
  {
#ifdef DMP_MATRIX_DEBUG
    printf("%s", s.c_str());
#endif
  }

  std::array<Scalar, n * m> data_;
};

//
// Rvalue-reference matrix multiplications
//
template<typename Scalar, int n, int m>
Matrix<Scalar, n, m> operator * (Matrix<Scalar, n, m>&& lhs, Scalar rhs)
{
  for (int i = 0; i < lhs.numElements(); i++)
    lhs.data_[i] *= rhs;
  return lhs;
}

// Template
using Vector4d = Matrix<double, 4, 1>;
using Matrix4d = Matrix<double, 4, 4>;

}

#endif //DMP_MATRIX_H
