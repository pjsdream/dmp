#include <core/comm/deserializer.h>

namespace dmp
{
Deserializer::Deserializer(const std::vector<char>& buffer)
    : buffer_(buffer)
{
}
}
