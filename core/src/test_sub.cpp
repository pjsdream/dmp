#include <core/comm/subscriber.h>

#include <thread>
#include <iostream>

int main()
{
  std::cout << "Testing subscriber\n";

  dmp::Subscriber subscriber("127.0.0.1");

  while (true)
  {
    using namespace std::chrono_literals;

    auto d = subscriber.receive<double>();

    if (d != nullptr)
    {
      std::cout << "received " << *d << "\n";
    }

    std::this_thread::sleep_for(16ms);
  }

  return 0;
}
