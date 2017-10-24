#ifndef DMP_ZMQ_SERIALIZER_H
#define DMP_ZMQ_SERIALIZER_H

#include <zmq.hpp>

#include <type_traits>

namespace dmp
{
class ZmqSerializer
{
public:
  ZmqSerializer() = delete;

  explicit ZmqSerializer(int size)
      : message_(size)
  {
    buffer_ = static_cast<char*>(message_.data());
  }

  zmq::message_t& getMessage()
  {
    return message_;
  }

  const zmq::message_t& getMessage() const
  {
    return message_;
  }

  template<typename T>
  ZmqSerializer& operator&(T&& v)
  {
    using type = typename std::remove_reference_t<T>;
    return *this << std::forward<type>(v);
  }

  template<typename T>
  ZmqSerializer& operator<<(T&& v)
  {
    using type = typename std::remove_reference_t<T>;
    return serialize(std::forward<type>(v),
                     std::is_same<std::integral_constant<bool,
                                                         std::is_compound<type>::value && !std::is_enum<type>::value>,
                                  std::true_type>());
  }

  template<typename T>
  ZmqSerializer& serialize(T&& v, std::true_type)
  {
    v.serialize(*this);
    return *this;
  }

  template<typename T>
  ZmqSerializer& serialize(T&& v, std::false_type)
  {
    union
    {
      char c[8];
      T value;
    } u{.value = v};

    for (int i = 0; i < 8; i++)
      buffer_[i] = u.c[i];
    buffer_ += 8;

    return *this;
  }

  // Specializations
  ZmqSerializer& operator<<(std::string&& s)
  {
    auto len = s.length();
    *this << len;

    for (auto i = 0; i < len; i++)
      buffer_[i] = s[i];

    auto aligned_len = (len + 7) / 8 * 8;
    for (auto i = len; i < aligned_len; i++)
      buffer_[i] = 0;

    buffer_ += aligned_len;
  }

  ZmqSerializer& operator<<(std::string& s)
  {
    auto len = s.length();
    *this << len;

    for (auto i = 0; i < len; i++)
      buffer_[i] = s[i];

    auto aligned_len = (len + 7) / 8 * 8;
    for (auto i = len; i < aligned_len; i++)
      buffer_[i] = 0;

    buffer_ += aligned_len;
  }

  template<typename T>
  ZmqSerializer& operator<<(std::vector<T>&& v)
  {
    auto len = v.size();
    *this << len;

    for (auto& element : v)
      *this << element;

    return *this;
  }

  template<typename T>
  ZmqSerializer& operator<<(std::vector<T>& v)
  {
    auto len = v.size();
    *this << len;

    for (auto& element : v)
      *this << element;

    return *this;
  }

private:
  zmq::message_t message_;
  char* buffer_;
};

class ZmqSerializerSizeEvaluator
{
public:
  template<typename T>
  ZmqSerializerSizeEvaluator& operator&(T&& v)
  {
    using type = typename std::remove_reference_t<T>;
    return *this << std::forward<type>(v);
  }

  template<typename T>
  ZmqSerializerSizeEvaluator& operator<<(T&& v)
  {
    using type = typename std::remove_reference_t<T>;
    std::integral_constant<bool, std::is_compound<type>::value && !std::is_enum<type>::value> tmp;
    addSize(std::forward<type>(v), std::is_same<std::integral_constant<bool, std::is_compound<type>::value && !std::is_enum<type>::value>, std::true_type>());
    return *this;
  }

  template<typename T>
  void addSize(T&& v, std::true_type)
  {
    v.serialize(*this);
  }

  template<typename T>
  void addSize(T&& v, std::false_type)
  {
    size_ += 8;
  }

  // Specializations
  ZmqSerializerSizeEvaluator& operator<<(std::string&& v)
  {
    auto len = v.length();
    auto aligned_len = (len + 7) / 8 * 8;
    size_ += 8 + static_cast<int>(aligned_len);
    return *this;
  }

  ZmqSerializerSizeEvaluator& operator<<(std::string& v)
  {
    auto len = v.length();
    auto aligned_len = (len + 7) / 8 * 8;
    size_ += 8 + static_cast<int>(aligned_len);
    return *this;
  }

  template<typename T>
  ZmqSerializerSizeEvaluator& operator<<(std::vector<T>&& v)
  {
    size_ += 8;
    for (auto& element : v)
      *this << element;
    return *this;
  }

  template<typename T>
  ZmqSerializerSizeEvaluator& operator<<(std::vector<T>& v)
  {
    size_ += 8;
    for (auto& element : v)
      *this << element;
    return *this;
  }

  int size() const
  {
    return size_;
  }

private:
  int size_{0};
};
}

#endif //DMP_ZMQ_SERIALIZER_H
