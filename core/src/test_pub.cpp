#include <core/comm/publisher.h>

#include <iostream>
#include <thread>
#include <chrono>

int main()
{
  std::cout << "Testing publisher\n";

  dmp::Publisher publisher("testing_topic", "127.0.0.1");

  using namespace std::chrono_literals;
  std::this_thread::sleep_for(50ms);

  for (int i=0; i<1000; i++)
  {
    auto d = static_cast<double>(i);
    std::cout << "[" << std::chrono::high_resolution_clock::now().time_since_epoch().count() << "] publishing " << d << "\n";

    publisher.publish(d);

    //std::this_thread::sleep_for(500ms);
  }

  return 0;
}
