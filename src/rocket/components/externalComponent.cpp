#include "externalComponent.hpp"
#include <iostream>

namespace Rocket{
    ExternalComponent::ExternalComponent(std::unique_ptr<Shapes::ExternalComponentShape> shape, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position):
    AeroComponent(nullptr, std::move(material), std::move(finish), name, position)
    {
        setShape( std::move(shape) );
    }

    std::vector< std::shared_ptr<AeroComponent> > ExternalComponent::aeroComponents() const {
        return _components;
    }

    std::vector<std::shared_ptr<Component>> ExternalComponent::components() const {
        std::vector<std::shared_ptr<Component>> compVec;
        for(auto comp = _components.begin(); comp != _components.end(); ++comp){
            compVec.push_back( std::dynamic_pointer_cast<Component>(*comp) );
        }
        return compVec;
    }

    std::shared_ptr<Component> ExternalComponent::findComponent(std::string id) const {
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

    double ExternalComponent::bodyRadius(double x) const {
        if(parent() != NULL){
            return parent()->bodyRadius(x);
        }
        return 0;
    }

    Shapes::ExternalComponentShape* ExternalComponent::shape() const {
        return _shape.get();
    }
    
    void ExternalComponent::setShape( std::unique_ptr<Shapes::AeroComponentShape> shape ){
        auto castedShapePointer = dynamic_cast<Shapes::ExternalComponentShape*>(shape.get());

        if(castedShapePointer != NULL){
            // if casting is successful the a unique pointer of the correct type is crated and the old one is released
            auto newShapeUniquePtr = std::unique_ptr<Shapes::ExternalComponentShape>(castedShapePointer);
            shape.release();
            return setShape( std::move(newShapeUniquePtr) );
        }
        // if the function has reached here it has failed
        std::cerr << "Invalid shape for external component\n"; // TODO: make this more descriptive

    }
    
    void ExternalComponent::setShape( std::unique_ptr<Shapes::ExternalComponentShape> shape ){
        _shape = std::move(shape);
        clearCaches();
    }

    AeroComponent* ExternalComponent::parent() const {
        if(_parent.expired()) return NULL;
        if(_parent.lock().get() == NULL) return NULL;
        return _parent.lock().get();
    }

    // need to override this so its operating on the correct parent
    void ExternalComponent::setParent( Component* parent ) {
        // add and remove component calls this so this cant be called by them
        auto castedPtr = dynamic_cast<AeroComponent*>(parent);
        if(castedPtr != NULL){
            if(parent == NULL){
                _parent.reset();
            } else {
                _parent = std::dynamic_pointer_cast<AeroComponent>(parent->shared_from_this());
            }
        } else {
            std::cerr << "invalid parent type for aero component";
        }
    }
}