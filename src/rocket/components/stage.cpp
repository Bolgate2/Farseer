#include "stage.hpp"
#include "bodyComponent.hpp"

namespace Rocket{
    std::string Stage::defaultName = "Sustainer";
    
    Stage::Stage(std::string name):
    AeroComponent(nullptr, nullptr, nullptr, name, Eigen::Vector3d::Zero())
    {}

    std::shared_ptr<Stage> Stage::create(Rocket* parent, std::string name){
        auto obj = std::shared_ptr<Stage>( new Stage(name) );
        if(parent != nullptr){
            parent->addComponent(obj.get());
        }
        return obj;
    }

    Rocket* Stage::parent() const {
        if(_parent.expired()) return nullptr;
        if(_parent.lock().get() == NULL) return nullptr;
        return _parent.lock().get();
    }

    // need to override this so its operating on the correct parent
    void Stage::setParent( Component* parent ) {
        // add and remove component calls this so this cant be called by them
        auto castedPtr = dynamic_cast<Rocket*>(parent);
        if(castedPtr != NULL){
            if(parent == NULL){
                _parent.reset();
            } else {
                _parent = std::dynamic_pointer_cast<Rocket>(parent->shared_from_this());
            }
        } else {
            std::cerr << "invalid parent type for Stage";
        }
    }

    std::shared_ptr<Component> Stage::findComponent(std::string id) const {
        auto comps = components();
        for(auto comp = comps.begin(); comp != comps.end(); ++comp){
            if( (*comp)->id() == id ) return (*comp)->shared_from_this();
        }
        std::cerr << "Unable to find component with id \"" << id << "\" in stage \"" << name << std::endl;
        return NULL;
    }

    std::vector< std::shared_ptr<AeroComponent> > Stage::aeroComponents() const {
        std::vector<std::shared_ptr<AeroComponent>> compVec;
        std::transform(
            _components.cbegin(), _components.cend(),
            std::back_inserter(compVec),
            [&](std::shared_ptr<BodyComponent> comp){ return std::dynamic_pointer_cast<AeroComponent>(comp); }//lambda function that dynamically casts the pointer and captures by reference
            );
        return compVec;
    }

    std::vector<std::shared_ptr<Component>> Stage::components() const {
        std::vector<std::shared_ptr<Component>> compVec;
        std::transform(
            _components.cbegin(), _components.cend(),
            std::back_inserter(compVec),
            [&](std::shared_ptr<BodyComponent> comp){ return std::dynamic_pointer_cast<Component>(comp); }//lambda function that dynamically casts the pointer and captures by reference
            );
        return compVec;
    }

    void Stage::addComponent(Component* component){
        auto compBodyCast = dynamic_cast<BodyComponent*>(component);
        if(compBodyCast != NULL){
            // removing from previous parent if applicable
            if(compBodyCast->parent() != NULL){
                compBodyCast->parent()->removeComponent(compBodyCast);
            }
            // setting new parent
            compBodyCast->setParent(this);
            // adding to component list
            _components.push_back(
                std::dynamic_pointer_cast<BodyComponent>(compBodyCast->shared_from_this())
                );
            clearCaches();
            return;
        }
        std::cerr << "Unable to add invalid component \"" << component->name  << "\" to component \"" << name << std::endl;
    }

    void Stage::removeComponent(Component* component){
        auto compExternalCast = dynamic_cast<BodyComponent*>(component);
        for(auto comp = _components.begin(); comp != _components.end(); ++comp){
            if(compExternalCast == (*comp).get()){
                // unsetting parent
                compExternalCast->setParent(NULL);
                // removing from component list if applicable
                _components.erase(comp);
                clearCaches();
                break;
            }
        }
        std::cerr << "Unable to find component \"" << component->name  << "\" to remove from component \"" << name << std::endl;
    }

    double Stage::referenceArea() const {
        // biggest is boss
        auto comps = aeroComponents();
        double refArea = 0;
        for(auto comp = comps.begin(); comp != comps.end(); comp++){
            refArea = std::max(refArea, (*comp)->referenceArea());
        }
        return refArea;
    }

    double Stage::referenceLength() const {
        // biggest is boss
        auto comps = aeroComponents();
        double refLen = 0;
        for(auto comp = comps.begin(); comp != comps.end(); comp++){
            refLen = std::max(refLen, (*comp)->referenceLength());
        }
        return refLen;
    }

    double Stage::wettedArea() const {
        auto comps = aeroComponents();
        double area = 0;
        for(auto comp = comps.begin(); comp != comps.end(); comp++){
            area += (*comp)->wettedArea();
        }
        return area;
    }
}