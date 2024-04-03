#include "component.hpp"

namespace Rocket{

UUIDv4::UUIDGenerator<std::mt19937_64> Component::_uuidGenerator = UUIDv4::UUIDGenerator<std::mt19937_64>();



}
