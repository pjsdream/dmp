#include <renderer/resource/resource.h>

namespace dmp
{
Resource::Resource(const std::shared_ptr<GlFunctions>& gl)
: gl_(gl)
{
}
}
