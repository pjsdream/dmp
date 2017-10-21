#include <core/comm/serializer.h>

#include <type_traits>

namespace dmp
{
Serializer::Serializer(std::vector<char>& buffer)
    : buffer_(buffer)
{
}
}
