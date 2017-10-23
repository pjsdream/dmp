#include <core/comm/subscriber.h>

#include <thread>
#include <iostream>

int main()
{
  std::cout << "Testing subscriber\n";

  dmp::Subscriber subscriber("127.0.0.1", "temp");

  while (true)
  {
    using namespace std::chrono_literals;

    double d;

    if (subscriber.receive(d))
    {
      std::cout << "received " << d << "\n";
    }

    std::this_thread::sleep_for(16ms);
  }

  return 0;
}
