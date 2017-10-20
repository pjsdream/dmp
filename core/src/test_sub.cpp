#include <core/comm/subscriber.h>

#include <thread>
#include <iostream>

int main()
{
  std::cout << "Testing subscriber\n";

  dmp::Context context;
  dmp::Subscriber<double> subscriber{context, "localhost"};

  while (true)
  {
    using namespace std::chrono_literals;

    double d;

    if (subscriber.receive(d))
    {
      std::cout << "received " << d << "\n";

    }
    else
    {
      std::cout << "not received\n";
    }

    std::this_thread::sleep_for(1s);
  }

  return 0;
}
