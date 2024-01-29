#include <numbers>
#include <cmath>
#include "bodyTube.hpp"

namespace Rocket{

BodyTube::BodyTube(HollowCylinder cyl)
{
    name = "toob";
    _shape = std::move(cyl);
}

}