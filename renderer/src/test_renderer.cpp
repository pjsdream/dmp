#include <renderer/renderer_ostream.h>
#include <renderer/request/request_mesh.h>

#include <iostream>

int main()
{
  dmp::RequestMesh request_mesh;
  request_mesh.action = dmp::RequestMesh::Action::Attach;
  request_mesh.filename = "/home/jaesungp/cpp_workspace/dmp/renderer/meshes/rect.obj";

  std::cout << "sending request to renderer\n";

  dmp::rout << request_mesh;

  std::cout << "sending request to renderer complete\n";

  return 0;
}
