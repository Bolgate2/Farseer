#include "rocket.hpp"
#include "stage.hpp"
#include "maths.hpp"
#include <iostream>

namespace Rocket{
    Rocket::Rocket(std::string name):AeroComponent(nullptr, nullptr, nullptr, name, Eigen::Vector3d::Zero()){}

    std::shared_ptr<Rocket> Rocket::create(std::string name){
        auto obj = std::shared_ptr<Rocket>( new Rocket(name) );
        return obj;
    }

    std::shared_ptr<Component> Rocket::findComponent(std::string id) const {
        auto comps = components();
        for(auto comp = comps.begin(); comp != comps.end(); ++comp){
            if( (*comp)->id() == id ) return (*comp)->shared_from_this();
        }
        std::cerr << "Unable to find component with id \"" << id << "\" in rocket \"" << name << std::endl;
        return NULL;
    }

    std::vector< std::shared_ptr<AeroComponent> > Rocket::aeroComponents() const {
        std::vector<std::shared_ptr<AeroComponent>> compVec;
        std::transform(
            _stages.cbegin(), _stages.cend(),
            std::back_inserter(compVec),
            [&](std::shared_ptr<Stage> comp){ return std::dynamic_pointer_cast<AeroComponent>(comp); }//lambda function that dynamically casts the pointer and captures by reference
            );
        return compVec;
    }

    std::vector<std::shared_ptr<Component>> Rocket::components() const {
        std::vector<std::shared_ptr<Component>> compVec;
        std::transform(
            _stages.cbegin(), _stages.cend(),
            std::back_inserter(compVec),
            [&](std::shared_ptr<Stage> comp){ return std::dynamic_pointer_cast<Component>(comp); }//lambda function that dynamically casts the pointer and captures by reference
            );
        return compVec;
    }

    void Rocket::addComponent(Component* component){
        auto compStageCast = dynamic_cast<Stage*>(component);
        if(compStageCast != NULL){
            // removing from previous parent if applicable
            if(compStageCast->parent() != NULL){
                compStageCast->parent()->removeComponent(compStageCast);
            }
            // setting new parent
            compStageCast->setParent(this);
            // adding to component list
            _stages.push_back(
                std::dynamic_pointer_cast<Stage>(compStageCast->shared_from_this())
                );
            clearCaches();
            return;
        }
        std::cerr << "Unable to add invalid component \"" << component->name  << "\" to component \"" << name << std::endl;
    }

    void Rocket::removeComponent(Component* component){
        auto compExternalCast = dynamic_cast<Stage*>(component);
        for(auto comp = _stages.begin(); comp != _stages.end(); ++comp){
            if(compExternalCast == (*comp).get()){
                // unsetting parent
                compExternalCast->setParent(NULL);
                // removing from component list if applicable
                _stages.erase(comp);
                clearCaches();
                break;
            }
        }
        std::cerr << "Unable to find component \"" << component->name  << "\" to remove from component \"" << name << std::endl;
    }

    double Rocket::referenceArea() const {
        // biggest is boss
        auto comps = aeroComponents();
        double refArea = 0;
        for(auto comp = comps.begin(); comp != comps.end(); comp++){
            refArea = std::max(refArea, (*comp)->referenceArea());
        }
        return refArea;
    }

    double Rocket::referenceLength() const {
        // biggest is boss
        auto comps = aeroComponents();
        double refLen = 0;
        for(auto comp = comps.begin(); comp != comps.end(); comp++){
            refLen = std::max(refLen, (*comp)->referenceLength());
        }
        return refLen;
    }

    double Rocket::wettedArea() const {
        auto comps = aeroComponents();
        double area = 0;
        for(auto comp = comps.begin(); comp != comps.end(); comp++){
            area += (*comp)->wettedArea();
        }
        return area;
    }

    Eigen::Matrix3d Rocket::inertia(double time) const {
        auto Io = AeroComponent::inertia(time);
        auto m = mass(time);
        auto CM = cm(time);
        auto Icm = Utils::parallel_axis_transform(Io, -CM, m, true);
        return Icm;
    }
    
    double Rocket::calculateLowestPoint() const {
        double lowestPoint = 0;
        auto comps = aeroComponents();
        for(auto comp = comps.begin(); comp != comps.end(); comp++){
            double compLow = (*comp)->calculateLowestPoint();
            if( compLow > lowestPoint) lowestPoint = compLow;
        }
        return lowestPoint;
    }

    double Rocket::calculateSurfaceDistanceTravelled(double x) const {
        double distTravelled = 0;
        auto comps = aeroComponents();
        for(auto comp = comps.begin(); comp != comps.end(); comp++){
            distTravelled += (*comp)->calculateSurfaceDistanceTravelled(x);
        }
        return distTravelled;
    }

    double Rocket::maxSurfaceDistanceTravelled() const {
        double distTravelled = 0;
        auto comps = aeroComponents();
        for(auto comp = comps.begin(); comp != comps.end(); comp++){
            distTravelled += (*comp)->maxSurfaceDistanceTravelled();
        }
        return distTravelled;
    }

    double Rocket::finenessRatio() const {
        return lowestPoint()/referenceLength();
    }

    Stage* Rocket::getLowestStage() const {
        Stage* lowComp = nullptr;
        double lowPoint = 0;
        auto comps = aeroComponents();
        for(auto comp = comps.begin(); comp != comps.end(); comp++){
            Stage* castedComp = dynamic_cast<Stage*>( (*comp).get() );
            if(castedComp != nullptr){
                double compLow = castedComp->calculateLowestPoint();
                if(lowComp == nullptr || compLow > lowPoint){
                    lowComp = castedComp;
                    lowPoint = compLow;
                }
            }
        }
        return lowComp;
    }

    double Rocket::CdbA(const double mach, const double time) const {
        auto lowStage = getLowestStage();
        if(lowStage == nullptr) return 0;
        return lowStage->CdbA(mach, time);
    }
}