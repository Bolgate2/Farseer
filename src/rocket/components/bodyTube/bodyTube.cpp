#include <numbers>
#include <cmath>
#include "bodyTube.hpp"

namespace Rocket{

BodyTube::BodyTube(HollowCylinder cyl)
{
    setName("toob");
    _shape = std::move(cyl);
}

}