#include "stage.hpp"

namespace Rocket{
    std::string Stage::defaultName = "Sustainer";
    
    Stage::Stage(std::string name):
    AeroComponent(nullptr, nullptr, nullptr, name, Eigen::Vector3d::Zero())
    {}
}