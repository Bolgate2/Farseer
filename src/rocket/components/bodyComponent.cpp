#include "bodyComponent.hpp"
#include "aeroComponent.hpp"
#include "internalComponent.hpp"
#include <vector>
#include <cmath>

namespace Rocket{
    std::string BodyComponent::defaultName = "Body Component";
    // constructors
    BodyComponent::BodyComponent(std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, BodyComponent* parent, std::string name, Eigen::Vector3d position):
    AeroComponent(std::move(material), std::move(finish), parent, name, position)
    {
        std::cout << "hi from bc constructor\n";
    }

    BodyComponent::BodyComponent(std::unique_ptr<Shapes::BodyComponentShape> shape, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, BodyComponent* parent, std::string name, Eigen::Vector3d position):
    AeroComponent(std::move(material), std::move(finish), parent, name, position)
    {
        setShape(std::move(shape));
        std::cout << "hi from bc constructor\n";
    }

    // cache stuff
    void BodyComponent::clearCaches(){
        AeroComponent::clearCaches();
        _bodyLiftCache.clear();
    }

    // aero stuff
    double BodyComponent::calculateBodyLift( double alpha ){
        return bodyLiftConst * planformArea()/referenceArea() * std::pow( std::sin(alpha), 2 );
    }

    double BodyComponent::calculateBodyLiftWithCache( double alpha ){
        if(!caching()) return calculateBodyLift( alpha );
        // check cache
        auto thisBodyLift = calculateBodyLift( alpha );
        _bodyLiftCache[alpha] = thisBodyLift;
        return thisBodyLift;
    }

    double BodyComponent::bodyLift( double alpha ){
        return calculateBodyLiftWithCache(alpha);
    }

    Eigen::Vector3d BodyComponent::bodyLiftCp(){
        auto pos = position();
        pos += Eigen::Vector3d{ planformCenter(),0,0 };
        return pos;
    }
    double BodyComponent::c_m_damp_Func(double length, double avgRadius, double omega, double v ){
        return 0.55 * (std::pow( length, 4 ) * avgRadius) / ( referenceArea() * referenceLength() ) * ( std::pow(omega, 2) / std::pow(v, 2) );
    }

    double BodyComponent::referenceLength(){
        return shape()->referenceLength();
    }

    // tree stuff
    BodyComponent* BodyComponent::parent(){
        return _parent;
    }

    std::vector<AeroComponent*> BodyComponent::aeroComponents(){
        return _externalComponents;
    }

    std::vector<Component*> BodyComponent::components(){
        // casting components
        std::vector<Component*> externalCompsAsComps(_externalComponents.begin(), _externalComponents.end());
        std::vector<Component*> internalCompsAsComps(_internalComponents.begin(), _internalComponents.end());
        // concatenating casted vectors
        internalCompsAsComps.insert(internalCompsAsComps.end(), externalCompsAsComps.begin(), externalCompsAsComps.end());
        return internalCompsAsComps;
    }

    Component* BodyComponent::findComponent(std::string id){
        for(auto comp = components().begin(); comp != components().end(); ++comp){
            if( (*comp)->id() == id ) return *comp;
        }
        return NULL;
    }

    void BodyComponent::addComponent(Component* component){
        // first try to cast the component to an internal comp
        auto compInternalCast = dynamic_cast<InternalComponent*>(component);
        if(compInternalCast != NULL){
            compInternalCast->setParent(this);
            _internalComponents.push_back(compInternalCast);
            clearCaches();
            return;
        }
        auto compExternalCast = dynamic_cast<ExternalComponent*>(component);
        if(compExternalCast != NULL){
            compInternalCast->setParent(this);
            _externalComponents.push_back(compExternalCast);
            clearCaches();
            return;
        }
        // if the function gets here, the component could not be added
        std::cerr << "Could not add component \'" << component->name << "\' to \'" << name << "\'\n";
    }

    void BodyComponent::removeComponent(Component* component){
        // first checking internal components
        auto compInternalCast = dynamic_cast<InternalComponent*>(component);
        if( compInternalCast != NULL ){
            for(auto comp = _internalComponents.begin(); comp != _internalComponents.end(); ++comp){
                if( component == (*comp) ){
                    (*comp)->setParent(NULL);
                    _internalComponents.erase(comp);
                    clearCaches();
                    return;
                }
            }
        }
        // then checking external components
        auto compExternalCast = dynamic_cast<ExternalComponent*>(component);
        if( compExternalCast != NULL){
            for(auto comp = _externalComponents.begin(); comp != _externalComponents.end(); ++comp){
                if( component == (*comp) ){
                    (*comp)->setParent(NULL);
                    _externalComponents.erase(comp);
                    clearCaches();
                    return;
                }
            }
        }
        // if the function reaches here it has failed
        std::cerr << "Could not find component \'" << component->name << "\' to remove from \'" << name << "\'\n";
    } // THIS MUST CLEAR CACHES

    // chape functions
    Shapes::BodyComponentShape* BodyComponent::shape() {
        return _shape.get();
    }
    
    void BodyComponent::setShape( std::unique_ptr<Shapes::AeroShape> shape ){
        // try to cast the underlying pointer to a pointer of the inherited class
        auto castedShapePointer = dynamic_cast<Shapes::BodyComponentShape*>(shape.get());

        if(castedShapePointer != NULL){
            // if casting is successful the a unique pointer of the correct type is crated and the old one is released
            auto newShapeUniquePtr = std::unique_ptr<Shapes::BodyComponentShape>(castedShapePointer);
            shape.release();
            return setShape( std::move(newShapeUniquePtr) );
        }
        // if the function has reached here it has failed
        std::cerr << "Invalid shape for body component\n"; // TODO: make this more descriptive
    }

    void BodyComponent::setShape( std::unique_ptr<Shapes::BodyComponentShape> shape ){
        _shape = std::move(shape);
    }

    double BodyComponent::bodyRadius(double x){
        return radius(x);
    }
    
    // getter and setter for shape length
    double BodyComponent::length(){
        return shape()->length();
    }

    void BodyComponent::setLength( double length){
        shape()->setLength( length );
        clearCaches();
    } // CLEAR CACHES
    // getter and setter for shape radius
    double BodyComponent::radius(){
        return shape()->radius();
    }

    double BodyComponent::radius( double x ){
        return shape()->radius( x-position().x() ); // converting from global coords to the shapes coords
    }

    void BodyComponent::setRadius( double radius ){
        shape()->setRadius( radius );
        clearCaches();
    }
    // getter and setter for shape thickness
    double BodyComponent::thickness(){
        return shape()->thickness();
    }
    void BodyComponent::setThickness( double thickness ){
        shape()->setThickness( thickness );
        clearCaches();
    }
    // area
    double BodyComponent::area(double x){
        return shape()->area( x - position().x() );
    }

    // component calcs
    // component calculations
    double BodyComponent::calculateMass(double time){
        auto density = material()->density;
        auto vol = shape()->volume();
        return density * vol;
    }

    
}