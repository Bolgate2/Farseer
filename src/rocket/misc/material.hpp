#ifndef MATERIAL_H_
#define MATERIAL_H_

#include <string>
#include <map>

namespace Rocket{
    struct Material{
        // variables
        std::string name;
        double density;

        // constructor
        Material(std::string name, double density);
    };

    extern const std::map<std::string, Material> defaultMaterials;
}

#endif