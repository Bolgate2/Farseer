#include "material.hpp"
#include <string>

namespace Rocket{
    Material::Material(std::string name, double density){
        this->name = name;
        this->density = density;
    }

    const std::map<std::string, Material> defaultMaterials = {
        {"Cardboard", Material("Cardboard", 680)},
        {"3D PLA", Material("3D PLA", 1250)},
        {"Plywood", Material("Plywood", 1250)}
    };
}