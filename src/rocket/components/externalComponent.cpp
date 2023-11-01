#include "externalComponent.hpp"
#include <iostream>

namespace Rocket{
    std::vector< std::shared_ptr<AeroComponent> > ExternalComponent::aeroComponents(){
        return _components;
    }

    std::vector<std::shared_ptr<Component>> ExternalComponent::components(){
        std::vector<std::shared_ptr<Component>> compVec;
        for(auto comp = _components.begin(); comp != _components.end(); ++comp){
            compVec.push_back( std::dynamic_pointer_cast<Component>(*comp) );
        }
        return compVec;
    }

    std::shared_ptr<Component> ExternalComponent::findComponent(std::string id){
        for(auto comp = components().begin(); comp != components().end(); ++comp){
            if( (*comp)->id() == id ) return (*comp)->shared_from_this();
        }
        std::cerr << "Unable to find component with id \"" << id << "\" in component \"" << name << std::endl;
        return NULL;
    }

    void ExternalComponent::addComponent(Component* component){
        auto compExternalCast = dynamic_cast<ExternalComponent*>(component);
        if(compExternalCast != NULL){
            // removing from previous parent if applicable
            if(compExternalCast->parent() != NULL){
                compExternalCast->parent()->removeComponent(compExternalCast);
            }
            // setting new parent
            compExternalCast->setParent(this);
            // adding to component list
            _components.push_back(
                std::dynamic_pointer_cast<ExternalComponent>(compExternalCast->shared_from_this())
                );
            clearCaches();
            return;
        }
        std::cerr << "Unable to add invalid component \"" << component->name  << "\" to component \"" << name << std::endl;
    }

    void ExternalComponent::removeComponent(Component* component){
        auto compExternalCast = dynamic_cast<ExternalComponent*>(component);
        for(auto comp = _components.begin(); comp != _components.end(); ++comp){
            if(compExternalCast == (*comp).get()){
                // unsetting parent
                compExternalCast->setParent(NULL);
                // removing from component list if applicable
                _components.erase(comp);
                clearCaches();
            }
        }
        std::cerr << "Unable to find component \"" << component->name  << "\" to remove from component \"" << name << std::endl;
    }

    double ExternalComponent::bodyRadius(double x){
        if(parent() != NULL){
            auto castedParent = std::dynamic_pointer_cast<AeroComponent>(parent()->shared_from_this());
            if( castedParent != NULL ) return castedParent->bodyRadius(x);
        }
        return 0;
    }
}