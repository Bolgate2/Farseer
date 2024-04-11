#pragma once
#include <memory>
#include <string>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

class Material{
    private:
        double _density;
    public:
        std::string name;
        double getDensity() { return _density; }
        void setDensity(double density) { _density = std::max(0.0, density); }
        
        json toJson(){
            return json {
                {"name", name},
                {"density", getDensity()}
            };
        }

        Material(std::string name, double density){
            this->name = name;
            setDensity(density);
        }

        static Material fromJson(json j){
            std::string nm = j.at("name");
            double dens = j.at("density");
            return Material(nm, dens);
        }
};
