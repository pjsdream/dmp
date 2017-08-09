#include <dmp/rendering/request/request_manager.h>
#include <dmp/rendering/request/request.h>

namespace dmp
{
void RequestManager::receiveRequest(const std::shared_ptr<Request>& request)
{
  std::lock_guard<std::mutex> lock{queue_mutex_};
  queue_.push_back(request);
}

std::vector<std::shared_ptr<Request>> RequestManager::getRequests()
{
  std::vector<std::shared_ptr<Request>> result;
  {
    std::lock_guard<std::mutex> lock{queue_mutex_};
    std::swap(result, queue_);
  }

  return result;
}
}
