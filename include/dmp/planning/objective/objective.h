#ifndef DMP_OBJECTIVE_H
#define DMP_OBJECTIVE_H

#include <type_traits>

namespace dmp
{
class Objective
{
public:
  virtual ~Objective() = default;

  template<typename T, typename = typename std::enable_if_t<std::is_base_of<Objective, T>::value>>
  T& as()
  {
    return static_cast<T>(*this);
  };

private:
};
}

#endif //DMP_OBJECTIVE_H
