#include <numbers>
#include <cmath>
#include "bodyTube.hpp"

namespace Rocket{

BodyTube::BodyTube(std::unique_ptr<HollowCylinder> cyl)
{
    name = "toob";
    _shape = std::move(cyl);
}

}