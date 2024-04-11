#pragma once
#include <memory>
#include <string>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

class Finish{
    private:
        double _roughness;
    public:
        std::string name;
        double getRoughness() { return _roughness; }
        void setRoughness(double roughness) { _roughness = std::max(0.0, roughness); }
        
        json toJson(){
            return json {
                {"name", name},
                {"roughness", getRoughness()}
            };
        }

        Finish(std::string name, double roughness){
            this->name = name;
            setRoughness(roughness);
        }

        static std::unique_ptr<Finish> fromJson(json j){
            std::string nm = j.at("name");
            double roughness = j.at("roughness");
            return std::move(
                std::make_unique<Finish>(nm, roughness)
            );
        }
};