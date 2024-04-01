#pragma once
#include <memory>
#include <nlohmann/json.hpp>
using json = nlohmann::json;


namespace Rocket {

    class Component : std::enable_shared_from_this<Component>{
        
        public:
            std::string name;

            // applies the properties in a JSON to this component
            virtual void fromJson(json j) = 0;
            // 
            virtual json toJson() = 0;
    };

    template<class T>
    Component* componentFromJson(json j){
        T comp = T();
        comp.fromJson(j);
        return comp;
    }

}