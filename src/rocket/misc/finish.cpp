#include "finish.hpp"
#include <string>
#include <cmath>

namespace Rocket{
    // defining constructor
    Finish::Finish(std::string name, double roughness){
        this->name = name;
        this->roughness = roughness;
    }

    const Finish regular_paint = Finish("Regular Paint", 60/(std::pow(10,6)));
}