#include "finish.hpp"
#include <string>
#include <cmath>

namespace Finish{
    // defining constructor
    Finish::Finish(std::string name, float roughness){
        this->name = name;
        this->roughness = roughness;

    }

    const Finish regular_paint = Finish("Regular Paint", 60/(std::pow(10,6)));
}