#ifndef DMP_DESERIALIZER_H
#define DMP_DESERIALIZER_H

#include <vector>
#include <string>

namespace dmp
{
class Deserializer
{
public:
  Deserializer() = delete;
  explicit Deserializer(const std::vector<char>& buffer);

  // Forward & operator with any type to corresponding << operators
  template<typename T>
  Deserializer& operator&(T&& t)
  {
    return *this >> std::forward<T>(t);
  }

  // For compound types, use the user-defined serialization functions
  template<typename T>
  Deserializer& deserialize(T& v, std::true_type)
  {
    v.serialize(*this);
    return *this;
  }

  template<typename T>
  Deserializer& deserialize(T& v, std::false_type)
  {
    // Operators for default data types
    constexpr auto size = sizeof(T);

    union FundamentalTypeDeconstructor
    {
      char c[size];
      T value;
    } u;

    for (int i = 0; i < size; i++)
      u.c[i] = buffer_[index_ + i];
    v = u.value;

    index_ += size;
    return *this;
  };

  template<typename T>
  Deserializer& operator>>(T& v)
  {
    return deserialize(v, std::is_compound<typename std::remove_reference_t<T>>());
  };

  // std::string argument
  Deserializer& operator>>(std::string& s)
  {
    decltype(s.length()) length;

    // As a result, index_ will increase by the number of bytes for string length
    *this >> length;

    s.resize(length);
    for (auto i = 0; i < length; i++)
      s[i] = buffer_[index_++];

    return *this;
  }

  // std::vector argument
  template<typename T>
  Deserializer& operator>>(std::vector<T>& v)
  {
    decltype(v.size()) length;
    *this >> length;
    v.resize(length);

    for (int i=0; i<length; i++)
      *this >> v[i];

    return *this;
  }

private:
  const std::vector<char>& buffer_;
  int index_{0};
};
}

#endif //DMP_DESERIALIZER_H
