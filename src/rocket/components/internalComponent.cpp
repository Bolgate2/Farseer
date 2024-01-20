#include "internalComponent.hpp"
#include <vector>
#include <memory>

namespace Rocket{
    std::string InternalComponent::defaultName = "Internal Component";
    // constructor
    InternalComponent::InternalComponent(std::string name, Eigen::Vector3d position):
    Component(name, position)
    {}

    // creator
    std::shared_ptr<InternalComponent> InternalComponent::create(Rocket::Component *parent, std::string name, Eigen::Vector3d position){
        auto obj = std::shared_ptr<InternalComponent>(
            new InternalComponent(name, position)
        );
        if(parent != nullptr){
            parent->addComponent(obj.get());
        }
        return obj;
    }

    // tree functions
    std::vector< std::shared_ptr<Component> > InternalComponent::components() const {
        std::vector<std::shared_ptr<Component>> compVec;
        std::transform(
            _components.cbegin(), _components.cend(),
            std::back_inserter(compVec),
            [&](std::shared_ptr<InternalComponent> comp){ return std::dynamic_pointer_cast<Component>(comp); }//lambda function that dynamically casts the pointer and captures by reference
            );
        return compVec;
    }

    std::shared_ptr<Component> InternalComponent::findComponent(std::string id) const {
        for(auto comp = _components.begin(); comp != _components.end(); ++comp){
            if( (*comp)->id() == id ) return (*comp)->shared_from_this();
        }
        std::cerr << "Unable to find component with id \"" << id << "\" in component \"" << name << std::endl;
        return NULL;
    }

    void InternalComponent::addComponent(Component* component){
        auto compInternalCast = dynamic_cast<InternalComponent*>(component);
        if(compInternalCast != NULL){
            // removing from previous parent if applicable
            if(compInternalCast->parent() != NULL){
                compInternalCast->parent()->removeComponent(compInternalCast);
            }
            // setting new parent
            compInternalCast->setParent(this);
            // adding to component list
            _components.push_back(
                std::dynamic_pointer_cast<InternalComponent>(compInternalCast->shared_from_this())
                );
            clearCaches();
            return;
        }
        std::cerr << "Unable to add invalid component \"" << component->name  << "\" to component \"" << name << std::endl;
    }

    void InternalComponent::removeComponent(Component* component){
        auto compInternalCast = dynamic_cast<InternalComponent*>(component);
        if(compInternalCast != NULL){
            for(auto comp = _components.begin(); comp != _components.end(); ++comp){
                if(compInternalCast == (*comp).get()){
                    // unsetting parent
                    compInternalCast->setParent(NULL);
                    // removing from component list if applicable
                    _components.erase(comp);
                    clearCaches();
                }
            }
        }
        std::cerr << "Unable to find component \"" << component->name  << "\" to remove from component \"" << name << std::endl;
    }
}