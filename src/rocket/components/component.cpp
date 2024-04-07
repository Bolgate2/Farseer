#include "component.hpp"

namespace Rocket{

UUIDv4::UUIDGenerator<std::mt19937_64> Component::_uuidGenerator = UUIDv4::UUIDGenerator<std::mt19937_64>();

Component::Component(std::string name, Eigen::Vector3d position){
    _id = _uuidGenerator.getUUID().str();
    this->name = name;
    setPosition(position);
}

// helper function, checks if a child is of a valid type for a parent
bool isCompValidChild(Component* parent, Component* child){
    // TODO: null checking
    auto allowedTypes = parent->allowedComponents();
    auto childType = child->type();
    return std::any_of(
            allowedTypes.cbegin(),
            allowedTypes.cend(),
            [childType](auto i){ return i == childType; }
        );
}

bool Component::setParent( Component* parent ){
    // null case
    if(parent == nullptr){
        _parent.reset();
        return true;
    }
    if(!isCompValidChild(parent, this)){
        return false;
    }
    _parent = parent->shared_from_this();
    return true;
}

bool Component::addComponent( Component* comp ){
    // null checking
    if(comp == nullptr){
        return false;
    }
    if(!isCompValidChild(this, comp)){
        return false;
    };
    // removing component from old parent if valid
    if(comp->parent() != nullptr){
        comp->parent()->removeComponent(comp);
    }
    comp->setParent(this);
    _components.push_back(comp->shared_from_this());
    return true;
}

std::shared_ptr<Component> Component::findComponent( std::string uuid ){
    auto comps = components();
    for(auto i = comps.cbegin(); i != comps.cend(); i++){
        if(i->get()->id() == uuid) return *i;
    }
    return nullptr;
}

bool Component::removeComponent( Component* comp ){
    if(comp == nullptr) return false;
    // searching for component
    for(auto c = _components.begin(); c != _components.end(); c++){
        if(c->get() == comp){
            // component found
            _components.erase(c);
            comp->setParent(nullptr);
            return true;
        }
    }
    // component not found
    return false;
}

bool Component::removeComponent( std::string uuid ){
    auto comp = findComponent(uuid);
    if(comp == nullptr){
        return false;
    }
    return removeComponent(comp.get());
}

json Component::toJson(){

    auto comps = components();
    std::vector<json> subcomp_jsons {};
    for(auto c = comps.begin(); c != comps.end(); c++){
        subcomp_jsons.push_back(
            c->get()->toJson()
        );
    }

    Eigen::Vector3d pos = getPosition();
    std::vector posVec { pos.x(), pos.y(), pos.z() };

    json comp_json = {
        {"name", name},
        {"component_type", type()},
        {"position", posVec},
        {"components", subcomp_jsons},
        {"properties", propertiesToJson()}
    };
    return comp_json;
}

void Component::fromJson(json j){
    name = j.at("name");
    std::vector<double> jsonPos = j.at("position");
    setPosition( Eigen::Vector3d {jsonPos[0],jsonPos[1],jsonPos[2]} );
    jsonToProperties(j);
    // TODO component creation
    
}

}
