#include <iostream>

#include <renderer/renderer_ostream.h>

int main()
{
  std::cout << "testing pcd\n";

  dmp::RendererOstream rout;
  dmp::RequestClear clear;
  rout << clear;
  rout.flush();

  return 0;
}
