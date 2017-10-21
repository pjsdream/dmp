#ifndef DMP_REQUEST_LIGHT_H
#define DMP_REQUEST_LIGHT_H

#include <dmp/rendering/request/request.h>
#include <dmp/rendering/light/light.h>

#include <Eigen/Dense>

namespace dmp
{
class RequestLight : public Request
{
public:
  enum class Action
  {
    Nothing,
    Set,
    Delete,
  };

  RequestLight();
  ~RequestLight() override = default;

  void setLight(int index, Light&& light);

  void deleteLight(int index);

  Action getAction();
  int getIndex();
  const Light& getLight();

private:
  Action action_;
  int index_;
  Light light_;
};
}

#endif //DMP_REQUEST_LIGHT_H
