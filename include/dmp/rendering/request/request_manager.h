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

  void receiveRequest(const std::shared_ptr<Request>& request);
  std::vector<std::shared_ptr<Request>> getRequests();

private:
  std::mutex queue_mutex_;
  std::vector<std::shared_ptr<Request>> queue_;
};
}

#endif //DMP_REQUEST_MANAGER_H
