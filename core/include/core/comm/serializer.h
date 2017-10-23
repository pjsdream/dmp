#ifndef DMP_SERIALIZER_H
#define DMP_SERIALIZER_H

#include <vector>
#include <string>
#include <type_traits>

namespace dmp
{
class Serializer
{
public:
  Serializer() = delete;
  explicit Serializer(std::vector<char>& buffer);

  // Forward & operator with any type to corresponding << operators
  template<typename T>
  Serializer& operator&(T&& t)
  {
    using type = typename std::remove_reference_t<T>;
    return *this << std::forward<type>(t);
  }

  // For compound types, use the user-defined serialization functions
  template<typename T>
  Serializer& serialize(T&& v, std::true_type)
  {
    return v.serialize(*this);
  }

  // Operators for primitive data types
  template<typename T>
  Serializer& serialize(T&& v, std::false_type)
  {
    constexpr auto size = sizeof(T);

    union
    {
      char c[size];
      T value;
    } u{.value = v};
    buffer_.insert(buffer_.end(), u.c, u.c + size);
    return *this;
  }

  // Universal reference
  template<typename T>
  Serializer& operator<<(T&& v)
  {
    using type = typename std::remove_reference_t<T>;
    return serialize(std::forward<type>(v), std::is_compound<type>());
  }

  // std::string argument
  Serializer& operator<<(std::string& s)
  {
    *this << s.length();
    buffer_.insert(buffer_.end(), std::begin(s), std::end(s));
    return *this;
  }

  Serializer& operator<<(std::string&& s)
  {
    *this << s.length();
    buffer_.insert(buffer_.end(), std::begin(s), std::end(s));
    return *this;
  }

  // std::vector argument
  template<typename T>
  Serializer& operator<<(std::vector<T>&& v)
  {
    *this << v.size();
    for (auto& element : v)
      *this << element;
    return *this;
  }

  template<typename T>
  Serializer& operator<<(std::vector<T>& v)
  {
    *this << v.size();
    for (auto& element : v)
      *this << element;
    return *this;
  }

private:
  std::vector<char>& buffer_;
};
}

#endif //DMP_SERIALIZER_H
