#include "component.hpp"
#include "bodyTube.hpp"

namespace Rocket{

std::shared_ptr<Component> componentFromJson(json j){
    const std::string type = j.at("component_type");
    std::shared_ptr<Component> comp = nullptr;

    // strings dont work with switch statements???
    if( type == COMPONENT_NAMES::BODY_TUBE ){
        comp = std::make_shared<BodyTube>();
    }
    else if( type == COMPONENT_NAMES::NOSECONE ){

    }
    else {
        comp = nullptr;
    }

    if(comp.get() == nullptr){
        // TODO: ERROR LOGGING
        return nullptr;
    }
    comp->applyJson(j);

    return comp;
}

}