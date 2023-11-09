# include "component.hpp"
#include "maths.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <string>
#include <fmt/core.h>

namespace Rocket{

    std::string Component::defaultName = "Component";
    UUIDv4::UUIDGenerator<std::mt19937_64> Component::_uuidGenerator = UUIDv4::UUIDGenerator<std::mt19937_64>();
    // creating default component behaviour here
    // default constructor

    Component::Component(std::string name, Eigen::Vector3d position){
        _id = _uuidGenerator.getUUID().str();
        this->name = name;
        setPosition(position);
    }

    // id
    std::string Component::id() const {
        return _id;
    }

    // cache clearing
    void Component::clearCaches(){
        _massCache.clear();
        _inertiaCache.clear();
        _cmCache.clear();
        _thrustCache.clear();
        _thrustPositionCache.clear();
        // clearing the cache of a child must clear the caches of their ancestors
        if(parent() != NULL){
            parent()->clearCaches();
        }
    }

    // parent
    Component* Component::parent() const {
        if(_parent.expired()) return NULL;
        if(_parent.lock().get() == NULL) return NULL;
        return _parent.lock().get();
    }

    void Component::setParent( Component* parent ) {
        // add and remove component calls this so this cant be called by them
        if(parent == NULL){
            _parent.reset();
        } else {
            _parent = parent->shared_from_this();
        }
    }

    // this function is not virtual as it is essentially a convenience wrapper
    void Component::removeComponent(std::string id) {
        auto component = findComponent(id);
        if (component != NULL){
            removeComponent(component.get()); // using pointer to ensure calling of underlying virtual function
        }
    };

    std::shared_ptr<Component> Component::root() {
        if(parent() == NULL){
            return shared_from_this();
        } else {
            return parent()->root();
        }
    }

    // position
    Eigen::Vector3d Component::position() const {
        return _position;
    }
    // this is not pure virtual as it has a default implementation
    void Component::setPosition(Eigen::Vector3d pos){
        _position = pos;
        clearCaches();
    }
    // these are not virtual as they are essentially convenience wrappers
    void Component::setPosition(double pos[3]){
        Eigen::Vector3d posVec(pos);
        return this->setPosition(posVec);
    }
    void Component::setPosition(double x, double y, double z){
        Eigen::Vector3d posVec(x,y,z);
        return this->setPosition(posVec);
    }

    double Component::calculateBurnoutTime(){
        double btime = 0;
        auto comps = this->components();
        for(auto comp = comps.cbegin(); comp != comps.cend(); ++comp){
            btime = std::max(btime, (*comp)->calculateBurnoutTime());
        }
        _burnoutTime = btime;
        return btime;
    }
    
    // mass
    double Component::calculateMass(double time) const {
        return _mass;
    }
    
    double Component::calculateMassWithComponents(double time) const {
        // handling override flags
        double thisMass = 0;
        if(_massOverride & OverrideFlags::OVERRIDEALL){
            return _mass;
        }
        else if(_massOverride & OverrideFlags::OVERRIDE){
            thisMass = _mass;
        }
        else{
            thisMass = this->calculateMass(time);
        }
        // summing component masses
        double compMasses = 0;
        auto comps = this->components();
        for(auto comp = comps.cbegin(); comp != comps.cend(); ++comp){
            compMasses += (*comp)->mass(time);
        }
        // summing component masses and this mass
        double totalMass = thisMass + compMasses;
        // adding calculated value to cache
        return totalMass;
    }

    double Component::mass(double time) const {
        if(!caching()) return calculateMassWithComponents(time);
        time = std::min(time, _burnoutTime); // clamping time to burnout
        // checking cache
        auto cacheKey = _massCache.find(time);
        if(cacheKey != _massCache.end()) return cacheKey->second;
        // returning if not found in cache
        auto m = calculateMassWithComponents(time);
        _massCache[time] = m;
        return m;
    }

    void Component::overrideMass(OverrideFlags flags){
        _massOverride = flags;
        clearCaches();
    }
    OverrideFlags Component::massOverriden(){
        return _massOverride;
    }
    void Component::setMass(double mass){
        _mass = mass;
        clearCaches();
    }
    void Component::overrideMass(OverrideFlags flags, double value){
        setMass(value);
        overrideMass(flags);
    }
    
    // inertia
    // calculates inertia about (0,0,0)
    Eigen::Matrix3d Component::calculateInertia(double time) const {
        return _inertia;
    }

    Eigen::Matrix3d Component::calculateInertiaWithComponents(double time) const {
        // comparing with flags
        Eigen::Matrix3d thisInertia = Eigen::Matrix3d::Zero();
        if(_inertiaOverride & OverrideFlags::OVERRIDEALL){
            return _inertia;
        }
        else if(_inertiaOverride & OverrideFlags::OVERRIDE){
            thisInertia = _inertia;
        }
        else{
            thisInertia = calculateInertia(time);
        }
        // summing component inertias like this requires them all to be calculated about (0,0,0)
        Eigen::Matrix3d compInertia = Eigen::Matrix3d::Zero();
        auto comps = components();
        for(auto comp = comps.cbegin(); comp != comps.cend(); ++comp){
            compInertia += (*comp)->inertia(time);
        }
        // summing this inertia and component inertias
        Eigen::Matrix3d totalInertia = thisInertia + compInertia;
        return totalInertia;
    }

    Eigen::Matrix3d Component::inertia(double time) const {
        if(!caching()) return calculateInertiaWithComponents(time);
        time = std::min(time, _burnoutTime); // clamping time to burnout
        // checking cache
        auto cacheKey = _inertiaCache.find(time);
        if( cacheKey != _inertiaCache.end()) return cacheKey->second;
        // returning if not found
        auto inert = calculateInertiaWithComponents(time);
        _inertiaCache[time] = inert;
        return inert;
    }

    void Component::overrideInertia(OverrideFlags flags){
        _inertiaOverride = flags;
        clearCaches();
    }

    OverrideFlags Component::inertiaOverriden(){
        return _inertiaOverride;
    }

    void Component::setInertia(Eigen::Matrix3d inertia){
        _inertia = inertia;
        clearCaches();
    }
    void Component::overrideInertia(OverrideFlags flags, Eigen::Matrix3d value){
        setInertia(value);
        overrideInertia(flags);
    }

    // cm, center of mass
    // by default cm is position
    Eigen::Vector3d Component::calculateCm(double time) const {
        return position();
    }

    Eigen::Vector3d Component::calculateCmWithComponents(double time) const {
        //comparing with flags
        Eigen::Vector3d thisCm = Eigen::Vector3d::Zero();
        if(_cmOverride & OverrideFlags::OVERRIDEALL){
            return _cm;
        }
        else if(_cmOverride & OverrideFlags::OVERRIDE){
            thisCm = _cm;
        }
        else{
            thisCm = calculateCm(time);
        }
        // getting weighted sum of component cms and total component mass
        Eigen::Vector3d weightedCompCm = Eigen::Vector3d::Zero();
        double totalCompMass = 0;
        auto comps = components();
        for(auto comp = comps.cbegin(); comp != comps.cend(); ++comp){
            weightedCompCm += (*comp)->mass(time)*(*comp)->cm(time);
            totalCompMass += (*comp)->mass(time);
        }
        // adding this cm*mass to weighted cm and this mass to mass 
        auto thisMass = calculateMass(time); // take mass sans components
        auto weightedCm = weightedCompCm + thisCm*thisMass;
        auto totalMass = totalCompMass + thisMass;
        // avoiding div by 0 error
        if(totalMass == 0){
            return Eigen::Vector3d::Zero();
        }
        // getting cm of this object and components
        auto calculatedCm = weightedCm/totalMass;
        return calculatedCm;
    }

    Eigen::Vector3d Component::cm(double time) const {
        if(!caching()) return calculateCmWithComponents(time);
        // checking cache
        auto cacheKey = _cmCache.find(time);
        if(cacheKey != _cmCache.end()) return cacheKey->second;
        // calculating if not found
        return calculateCmWithComponents(time);
    }

    void Component::overrideCm(OverrideFlags flags){
        _cmOverride = flags;
        clearCaches();
    }

    OverrideFlags Component::cmOverriden(){
        return _cmOverride;
    }

    void Component::setCm(Eigen::Vector3d cm){
        _cm = cm;
        clearCaches();
    }

    void Component::overrideCm(OverrideFlags flags, Eigen::Vector3d value){
        setCm(value);
        overrideCm(flags);
    }

    // thrust
    Eigen::Vector3d Component::calculateThrust(double time) const {
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d Component::calculateThrustWithComponents(double time) const {
        Eigen::Vector3d compThrust = Eigen::Vector3d::Zero();
        auto comps = components();
        for(auto comp = comps.cbegin(); comp != comps.cend(); ++comp){
            compThrust += (*comp)->thrust(time);
        }
        auto totalThrust = compThrust + calculateThrust(time);
        return totalThrust;
    }
    
    Eigen::Vector3d Component::thrust(double time) const {
        if(!caching()) return calculateThrustWithComponents(time);
        auto cacheKey = _thrustCache.find(time);
        if(cacheKey != _thrustCache.end()) return cacheKey->second;
        auto th = calculateThrustWithComponents(time);
        _thrustCache[time] = th;
        return th;
    }

    //thrust position
    Eigen::Vector3d Component::calculateThrustPosition(double time) const {
        return cm(time);
    }

    Eigen::Vector3d Component::calculateThrustPositionWithComponents(double time) const {
        //initializing vectors
        Eigen::Vector3d weightedThrustPos = Eigen::Vector3d::Zero();
        double thrustMagnitude = 0;
        auto comps = components();
        for(auto comp = comps.cbegin(); comp != comps.cend(); ++comp){
            auto tMag = (*comp)->thrust(time).norm();
            weightedThrustPos += tMag * (*comp)->thrustPosition(time);
            thrustMagnitude += tMag;
        }
        auto tMag = calculateThrust(time).norm();
        thrustMagnitude += tMag;
        // thrust position is zero by default
        Eigen::Vector3d thrustPos = Eigen::Vector3d::Zero();
        if(thrustMagnitude > 0){
            // setting thrust position if thrust is more than zero, this avoids divide by 0 errors
            weightedThrustPos += tMag*calculateThrustPosition(time);
            thrustPos = weightedThrustPos/thrustMagnitude;
        }
        return thrustPos;
    }

    Eigen::Vector3d Component::thrustPosition(double time) const {
        if(!caching()) return calculateThrustPositionWithComponents(time);
        auto cacheKey = _thrustPositionCache.find(time);
        if( cacheKey != _thrustPositionCache.end() ) return cacheKey->second;
        return calculateThrustPositionWithComponents(time);
    }

    bool Component::caching() const {
        return _caching;
    }

    void Component::setCaching( bool toCache ){
        _caching = toCache;
    }
    
    void Component::setAllCaching( bool toCache ){
        setCaching(toCache);
        auto comps = components();
        for(auto comp = comps.begin(); comp != comps.end(); ++comp){
            (*comp)->setAllCaching(toCache);
        }
    }

    int Component::height() const {
        int val = 0;
        auto comps = components();
        if(comps.empty()) return val;

        for(auto comp = comps.begin(); comp != comps.end(); ++comp){
            int compHeight = (*comp)->height();
            if( compHeight > val) val = compHeight;
        }
        val++;
        return val;
    }

    void Component::printComponentTree(bool header) const {
        std::string repr = componentTreeRepr(header);
        fmt::print(repr);
        //std::cout << repr; // this stuffs up unicode characters
    }

    int Component::_indentLen = 4;
    int Component::_maxNameLen = 15;
    int Component::_massLen = 9;
    int Component::_massPrecision = 4;

    std::string Component::componentTreeRepr(bool header)const {
        auto h = this->height();
        const int maxPfxLen = _indentLen*h;

        std::string repr = "";
        std::string fmtStr = "|{:^" + std::to_string(maxPfxLen+_maxNameLen) + "}|{:^" + std::to_string(_massLen) +"}|{:^30}|\n";
        if(header) repr += fmt::format(fmtStr, "Name", "Mass (kg)", "CM [x,y,z] (m)");
        
        repr += componentTreeRepr("", "", this->height());
        return repr;
    }

    std::string Component::componentTreeRepr( std::string prefix, std::string childPrefix, int treeHeight) const {
        const int maxPfxLen = _indentLen*treeHeight;
        std::string fmtStr = " {:<" + std::to_string(maxPfxLen+_maxNameLen) + "} {:< " + std::to_string(_massLen) + "." + std::to_string(_massPrecision) + "f} {:^30} \n";
        std::stringstream cmstream;
        cmstream << "[" << cm(0).transpose() << "]";
        std::string repr = fmt::format(fmtStr, prefix + name, mass(0), cmstream.str());
        if(!components().empty()){
            auto comps = components();
            for(auto comp = comps.cbegin(); comp != comps.cend(); comp++){
                if((*comp).get() != comps.back().get()){
                    repr += (*comp)->componentTreeRepr( childPrefix + "├── ", childPrefix + "│   ", treeHeight);
                } else {
                    repr += (*comp)->componentTreeRepr( childPrefix + "└── ", childPrefix + "    ", treeHeight);
                }
            }
        }
        return repr;
    }

}