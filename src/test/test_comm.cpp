#include <dmp/comm/node.h>
#include <dmp/comm/publisher.h>
#include <dmp/comm/subscriber.h>

class IntMsg
{
public:
  explicit IntMsg(int x) : data{x}
  {}

  int data;
};

class Node1 : public dmp::Node
{
public:
  explicit Node1(const std::string& name)
      : Node(name)
  {
  }

  auto& getPublisher()
  { return publisher_; }

protected:
  void run() override
  {
    print("running node1\n");

    for (int i = 0; i < 10; i++)
    {
      print("sending message %d\n", i);
      publisher_.publish(IntMsg(i));
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

private:
  dmp::Publisher<IntMsg> publisher_;
};

class Node2 : public dmp::Node
{
public:
  explicit Node2(const std::string& name)
      : Node(name)
  {
  }

  auto& getSubscriber()
  { return subscriber_; }

protected:
  void run() override
  {
    print("running node2\n");

    while (true)
    {
      print("subscribing\n");
      // TODO: refactoring comm
      //auto values = subscriber_.popAll();
      std::vector<std::unique_ptr<IntMsg>> values;
      print("received %d messages\n", values.size());
      for (const auto& value : values)
      {
        print("received %d\n", value->data);

        if (value->data == 9)
        {
          print("terminating %s\n", getNodeName().c_str());
          return;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }

private:
  dmp::Subscriber<IntMsg> subscriber_;
};

int main(int argc, char** argv)
{
  setbuf(stdin, NULL);
  setbuf(stdout, NULL);

  auto node1 = std::make_shared<Node1>("node1");
  auto node2 = std::make_shared<Node2>("node2");

  // TODO: connect publisher and subscriber

  // TODO: run threads
  return 0;
}
