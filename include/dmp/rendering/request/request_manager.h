#ifndef DMP_REQUEST_MANAGER_H
#define DMP_REQUEST_MANAGER_H

#include <memory>
#include <mutex>
#include <vector>

namespace dmp
{
class Request;
class RequestManager
{
public:
  RequestManager() = default;
  ~RequestManager() = default;

  void addRequest(Request&& request);
  void addRequests(std::vector<Request>&& requests);
  void pullRequests(std::vector<Request>& result);

private:
  std::mutex queue_mutex_;
  std::vector<Request> queue_;
};
}

#endif //DMP_REQUEST_MANAGER_H
