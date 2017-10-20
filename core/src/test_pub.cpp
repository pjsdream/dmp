#include <core/comm/publisher.h>

#include <iostream>
#include <thread>

int main()
{
  std::cout << "Testing publisher\n";

  dmp::Context context;
  dmp::Publisher<double> publisher{context};

  using namespace std::chrono_literals;
  std::this_thread::sleep_for(1s);

  for (int i=0; i<100; i++)
  {

    auto d = static_cast<double>(i);
    std::cout << "publishing " << d << "\n";

    publisher.publish(d);

    std::this_thread::sleep_for(500ms);
  }

  return 0;
}
