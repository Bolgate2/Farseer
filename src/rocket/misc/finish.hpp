#ifndef FINISH_H_
#define FINISH_H_

#include <string>
#include <cmath>

namespace Rocket{

    struct Finish{
        // structs are the same as classes except things are public by default instead of private
        // variables
        std::string name;
        float roughness;

        // contructors
        Finish(std::string name, float roughness);
        
        // default types
        Finish * default_paint;

    };

    extern const Finish regular_paint;

}

#endif