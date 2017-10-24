#ifndef DMP_ZMQ_SUBSCRIBER_H
#define DMP_ZMQ_SUBSCRIBER_H

#include <zmq.hpp>

namespace dmp
{
class ZmqDeserializer
{
public:
  ZmqDeserializer() = delete;

  explicit ZmqDeserializer(const zmq::message_t& message)
      : buffer_(message.data<char>())
  {
  }

  template<typename T>
  ZmqDeserializer& operator&(T&& v)
  {
    using type = typename std::remove_reference_t<T>;
    return *this >> v;
  }

  template<typename T>
  ZmqDeserializer& operator>>(T&& v)
  {
    using type = typename std::remove_reference_t<T>;
    return deserialize(std::forward<type>(v), std::is_same<std::integral_constant<bool, std::is_compound<type>::value && !std::is_enum<type>::value>, std::true_type>());
  }

  template<typename T>
  ZmqDeserializer& deserialize(T&& v, std::true_type)
  {
    v.serialize(*this);
    return *this;
  }

  template<typename T>
  ZmqDeserializer& deserialize(T&& v, std::false_type)
  {
    union
    {
      char c[8];
      T value;
    } u;
    memcpy(u.c, buffer_, 8);
    v = u.value;
    buffer_ += 8;

    return *this;
  }

  // Specializations
  ZmqDeserializer& operator>>(std::string& v)
  {
    decltype(v.size()) len;
    *this >> len;
    v.resize(len);

    auto aligned_len = (len + 7) / 8 * 8;
    for (int i=0; i<len; i++)
      v[i] = buffer_[i];
    buffer_ += aligned_len;

    return *this;
  }

  template<typename T>
  ZmqDeserializer& operator>>(std::vector<T>& v)
  {
    decltype(v.size()) len;
    *this >> len;
    v.resize(len);

    for (auto& element : v)
      *this >> element;

    return *this;
  }

private:
  const char* buffer_;
};
}

#endif //DMP_ZMQ_SUBSCRIBER_H
