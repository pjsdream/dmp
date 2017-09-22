#ifndef DMP_MESSAGE_H
#define DMP_MESSAGE_H

namespace dmp
{
class Message
{
public:
  Message() = default;
  virtual ~Message() = default;

  Message(const Message& rhs) = default;
  Message& operator=(const Message& rhs) = default;

  Message(Message&& rhs) = default;
  Message& operator=(Message&& rhs) = default;

private:
};
}

#endif //DMP_MESSAGE_H
