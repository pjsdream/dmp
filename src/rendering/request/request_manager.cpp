#include <dmp/rendering/request/request_manager.h>
#include <dmp/rendering/request/request.h>

namespace dmp
{
void RequestManager::addRequest(Request&& request)
{
  std::lock_guard<std::mutex> lock{queue_mutex_};
  queue_.push_back(std::move(request));
}

void RequestManager::addRequests(std::vector<Request>&& requests)
{
  std::lock_guard<std::mutex> lock{queue_mutex_};

  if (queue_.empty())
  {
    queue_ = std::move(requests);
  }
  else
  {
    queue_.reserve(queue_.size() + requests.size());
    queue_.insert(queue_.cend(), std::make_move_iterator(requests.begin()), std::make_move_iterator(requests.end()));
  }
}

void RequestManager::pullRequests(std::vector<Request>& result)
{
  result.clear();

  {
    std::lock_guard<std::mutex> lock{queue_mutex_};
    std::swap(result, queue_);
  }
}
}
