#include <core/comm/subscriber.h>

#include <thread>
#include <iostream>

int main()
{
  std::cout << "Testing subscriber\n";

  dmp::Subscriber subscriber("testing_topic", "127.0.0.1");

  while (true)
  {
    using namespace std::chrono_literals;

    double d;

    if (subscriber.receive(d))
    {
      std::cout << "[" << std::chrono::high_resolution_clock::now().time_since_epoch().count() << "] received " << d << "\n";
    }

    std::this_thread::sleep_for(16ms);
  }

  return 0;
}
