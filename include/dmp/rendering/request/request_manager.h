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
  virtual ~RequestManager() = default;

  void addRequest(std::unique_ptr<Request> request);
  void addRequests(std::vector<std::unique_ptr<Request>>&& request);

  std::vector<std::unique_ptr<Request>> pullRequests();

private:
  std::mutex queue_mutex_;
  std::vector<std::unique_ptr<Request>> queue_;
};
}

#endif //DMP_REQUEST_MANAGER_H
