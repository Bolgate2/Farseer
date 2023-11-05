#include "aeroComponent.hpp"
#include "nanValues.hpp"
#include "maths.hpp"
#include <cmath>

namespace Rocket{
    // constructors
    std::string AeroComponent::defaultName = "Aero Component";

    AeroComponent::AeroComponent(std::unique_ptr<Shapes::AeroComponentShape> shape, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position):
    Component(name, position)
    {
        setShape( std::move(shape) );
        setMaterial( std::move(material) );
        setFinish(std::move(finish));
    }
    
    // clearing caches
    void AeroComponent::clearCaches(){
        clearC_n_aCache();
        clearC_m_aCache();
        clearCpCache();
        clearC_m_dampCache();
        Component::clearCaches();
    }

    void AeroComponent::clearC_n_aCache(){
        // cache is like _c_n_aCache[mach][alpha][gamma] = value;
        for(auto alphaCache = _c_n_aCache.begin(); alphaCache != _c_n_aCache.end(); ++alphaCache){
            for(auto gammaCache = (*alphaCache).second.begin(); gammaCache != (*alphaCache).second.end(); ++gammaCache){
                (*gammaCache).second.clear(); // clearing _c_n_aCache[mach][alpha][X]
            }
            (*alphaCache).second.clear(); // clearing _c_n_aCache[mach][X]
        }
        _c_n_aCache.clear(); // clearing _c_n_aCache[X]
    }

    void AeroComponent::clearC_m_aCache(){
        // cache is like _c_m_aCache[mach][alpha][gamma] = value;
        for(auto alphaCache = _c_m_aCache.begin(); alphaCache != _c_m_aCache.end(); ++alphaCache){
            for(auto gammaCache = (*alphaCache).second.begin(); gammaCache != (*alphaCache).second.end(); ++gammaCache){
                (*gammaCache).second.clear(); // clearing _c_m_aCache[mach][alpha][X]
            }
            (*alphaCache).second.clear(); // clearing _c_m_aCache[mach][X]
        }
        _c_m_aCache.clear(); // clearing _c_m_aCache[X]
    }

    void AeroComponent::clearCpCache(){
        // cache is like _cpCache[mach][alpha][gamma] = value;
        for(auto alphaCache = _cpCache.begin(); alphaCache != _cpCache.end(); ++alphaCache){
            for(auto gammaCache = (*alphaCache).second.begin(); gammaCache != (*alphaCache).second.end(); ++gammaCache){
                (*gammaCache).second.clear(); // clearing _cpCache[mach][alpha][X]
            }
            (*alphaCache).second.clear(); // clearing _cpCache[mach][X]
        }
        _cpCache.clear(); // clearing _cpCache[X]
    }

    void AeroComponent::clearC_m_dampCache(){
        // cache is like _cpCache[mach][alpha][gamma] = value;
        for(auto omegaCache = _c_m_dampCache.begin(); omegaCache != _c_m_dampCache.end(); ++omegaCache){
            for(auto vCache = (*omegaCache).second.begin(); vCache != (*omegaCache).second.end(); ++omegaCache){
                (*vCache).second.clear(); // clearing _cpCache[mach][alpha][X]
            }
            (*omegaCache).second.clear(); // clearing _cpCache[mach][X]
        }
        _c_m_dampCache.clear(); // clearing _cpCache[X]
    }

    // adding cache elems
    void AeroComponent::createC_n_aMapping( double value, double mach, double alpha, double gamma){
        _c_n_aCache[mach][alpha][gamma] = value;
    }
    void AeroComponent::createC_m_aMapping( double value, double mach, double alpha, double gamma){
        _c_m_aCache[mach][alpha][gamma] = value;
    }
    void AeroComponent::createCpMapping( Eigen::Vector3d value, double mach, double alpha, double gamma){
        _cpCache[mach][alpha][gamma] = value;
    }
    void AeroComponent::createC_m_dampMapping(double value, double x, double omega, double v){
        _c_m_dampCache[x][omega][v] = value;
    }

    // checking that args exist in cache
    bool AeroComponent::c_n_aExists(double mach, double alpha, double gamma){
        auto alphaCacheKey = _c_n_aCache.find(mach);
        if(alphaCacheKey != _c_n_aCache.end()){
            auto alphaCache = _c_n_aCache[mach];
            auto gammaCacheKey = alphaCache.find(alpha);
            if(gammaCacheKey != alphaCache.end()){
                auto gammaCache = alphaCache[alpha];
                auto valKey = gammaCache.find(gamma);
                if(valKey != gammaCache.end()){
                    return true;
                }
            }
        }
        return false;
    }

    bool AeroComponent::c_m_aExists(double mach, double alpha, double gamma){
        auto alphaCacheKey = _c_m_aCache.find(mach);
        if(alphaCacheKey != _c_m_aCache.end()){
            auto alphaCache = _c_m_aCache[mach];
            auto gammaCacheKey = alphaCache.find(alpha);
            if(gammaCacheKey != alphaCache.end()){
                auto gammaCache = alphaCache[alpha];
                auto valKey = gammaCache.find(gamma);
                if(valKey != gammaCache.end()){
                    return true;
                }
            }
        }
        return false;
    }

    bool AeroComponent::cpExists(double mach, double alpha, double gamma){
        auto alphaCacheKey = _cpCache.find(mach);
        if(alphaCacheKey != _cpCache.end()){
            auto alphaCache = _cpCache[mach];
            auto gammaCacheKey = alphaCache.find(alpha);
            if(gammaCacheKey != alphaCache.end()){
                auto gammaCache = alphaCache[alpha];
                auto valKey = gammaCache.find(gamma);
                if(valKey != gammaCache.end()){
                    return true;
                }
            }
        }
        return false;
    }

    bool AeroComponent::c_m_dampExists(double x, double omega, double v){
        auto omegaCacheKey = _c_m_dampCache.find(x);
        if(omegaCacheKey != _c_m_dampCache.end()){
            auto omegaCache = _c_m_dampCache[x];
            auto vCacheKey = omegaCache.find(omega);
            if(vCacheKey != omegaCache.end()){
                auto vCache = omegaCache[omega];
                auto valKey = vCache.find(v);
                if(valKey != vCache.end()){
                    return true;
                }
            }
        }
        return false;
    }

    // component and cache calculations of elems
    //C_n
    double AeroComponent::calculateC_n_aWithComponents( double mach, double alpha, double gamma){
        auto thisC_n_a = this->calculateC_n_a(mach, alpha, gamma);
        auto aeroComps = aeroComponents();
        for(auto aeroComp = aeroComps.begin(); aeroComp != aeroComps.end(); ++aeroComp){
            auto compC_n_a = (*aeroComp)->c_n_a(mach, alpha, gamma);
            auto compAref = (*aeroComp)->referenceArea();
            auto compAdjC_n_a = compC_n_a*compAref/referenceArea();
            thisC_n_a += compAdjC_n_a;
        }
        return thisC_n_a;
    }

    double AeroComponent::calculateC_n_aWithCache( double mach, double alpha, double gamma){
        if(!caching()) return calculateC_n_aWithComponents(mach, alpha, gamma);
        if(c_n_aExists(mach, alpha, gamma)) return _c_n_aCache[mach][alpha][gamma];
        // do interpolation here
        auto thisC_n_a = calculateC_n_aWithComponents(mach, alpha, gamma);
        createC_n_aMapping(thisC_n_a, mach, alpha, gamma);
        return thisC_n_a;
    }

    double AeroComponent::c_n_a( double mach, double alpha, double gamma){
        return calculateC_n_aWithCache(mach, alpha, gamma);
    }

    double AeroComponent::c_n( double mach, double alpha, double gamma){
        return c_n_a(mach, alpha, gamma)*alpha;
    }

    //C_m
    double AeroComponent::calculateC_m_aAtOrigin( double mach, double alpha, double gamma){
        // using ORK eq 3.3, dividing both sides by alpha
        /* just using the components coefficients rather than their children as only this comp
         * is relevant here
         */
        auto thisC_m_a = calculateC_m_a(mach, alpha, gamma);
        auto thisC_n_a = calculateC_n_a(mach, alpha, gamma);
        auto dist = position().x(); // because this is a moment about the y axis caused by forces in the z direction, only the x distance matters
        auto newC_m_a = (thisC_m_a*referenceLength() - thisC_n_a*dist) / referenceLength();
        return newC_m_a;
    }

    double AeroComponent::calculateC_m_aWithComponents( double mach, double alpha, double gamma){
        auto thisC_m_a = calculateC_m_aAtOrigin(mach, alpha, gamma);
        auto aeroComps = aeroComponents();
        for(auto aeroComp = aeroComps.begin(); aeroComp != aeroComps.end(); ++aeroComp){
            auto compC_m_a = (*aeroComp)->c_m_a(mach, alpha, gamma);
            auto compAref = (*aeroComp)->referenceArea();
            auto compDref = (*aeroComp)->referenceLength();
            auto compAdjC_m_a = compC_m_a* (compAref*compDref) / (referenceArea()*referenceLength());
            thisC_m_a += compAdjC_m_a;
        }
        return thisC_m_a;
    }

    double AeroComponent::calculateC_m_aWithCache( double mach, double alpha, double gamma){
        if(!caching()) return calculateC_m_aWithComponents(mach, alpha, gamma);
        if(c_m_aExists(mach, alpha, gamma)) return _c_m_aCache[mach][alpha][gamma];
        // do interpolation here
        auto thisC_m_a = calculateC_m_aWithComponents(mach, alpha, gamma);
        createC_m_aMapping(thisC_m_a, mach, alpha, gamma);
        return thisC_m_a;
    }

    double AeroComponent::c_m_a( double mach, double alpha, double gamma){
        return calculateC_m_aWithCache(mach, alpha, gamma);
    }

    double AeroComponent::c_m( double mach, double alpha, double gamma){
        return c_m_a(mach, alpha, gamma) * alpha;
    }

    //CP
    Eigen::Vector3d AeroComponent::calculateCpWithComponents( double mach, double alpha, double gamma){
        // getting cp using ORK eq 3.29
        // because this only considers C_n_a it will only affect the x position of cp, can be improved in future
        Eigen::Vector3d weightedCpSum = Eigen::Vector3d::Zero();
        double c_n_aSum = 0;
        auto aeroComps = aeroComponents();
        for(auto aeroComp = aeroComps.begin(); aeroComp != aeroComps.end(); ++aeroComp){
            std::cout << "hi" << std::endl;
            //std::cout << "comp name " << (*aeroComp)->name << std::endl;
            auto compC_n_a = (*aeroComp)->c_n_a(mach, alpha, gamma);
            auto compCP = (*aeroComp)->cp(mach, alpha, gamma);
            weightedCpSum += compC_n_a * compCP;
            c_n_aSum += compC_n_a;
        }
        auto thisC_n_a = calculateC_n_a(mach, alpha, gamma);
        auto thisCp = calculateCp(mach, alpha, gamma);
        c_n_aSum += thisC_n_a; //cant use c_n_a like with components as c_n_a would include those components
        weightedCpSum += thisCp * thisC_n_a ;
        if(c_n_aSum == 0){
            return Eigen::Vector3d::Zero();
        }
        return weightedCpSum/c_n_aSum;
    }

    Eigen::Vector3d AeroComponent::calculateCpWithCache( double mach, double alpha, double gamma){
        if(!caching()) return calculateCpWithComponents(mach, alpha, gamma);
        if(cpExists(mach, alpha, gamma)) return _cpCache[mach][alpha][gamma];
        auto thisCp = calculateCpWithComponents(mach, alpha, gamma);
        createCpMapping(thisCp, mach, alpha, gamma);
        return thisCp;
    }

    Eigen::Vector3d AeroComponent::cp( double mach, double alpha, double gamma){
        return calculateCpWithCache(mach, alpha, gamma);
    }

    //C_m_damp
    double AeroComponent::calculateC_m_dampWithComponents(double x, double omega, double v){
        auto thisC_m_damp = calculateC_m_damp(x, omega, v);
        auto aeroComps = aeroComponents();
        for(auto aeroComp = aeroComps.begin(); aeroComp != aeroComps.end(); ++aeroComp){
            auto compC_m_damp = (*aeroComp)->c_m_damp(x, omega, v);
            auto compAref = (*aeroComp)->referenceArea();
            auto compDref = (*aeroComp)->referenceLength();
            auto compAdjC_m_damp = compC_m_damp* (compAref*compDref) / (referenceArea()*referenceLength());
            thisC_m_damp += compAdjC_m_damp;
        }
        return thisC_m_damp;
    }

    double AeroComponent::calculateC_m_dampWithCache(double x, double omega, double v){
        if(!caching()) return calculateC_m_dampWithComponents(x, omega, v);
        if(c_m_dampExists(x, omega, v)) return _c_m_dampCache[x][omega][v];
        auto thisC_m_damp = calculateC_m_dampWithComponents(x, omega, v);
        createC_m_dampMapping(thisC_m_damp, x, omega, v);
        return thisC_m_damp;
    }

    double AeroComponent::c_m_damp(double x, double omega, double v){
        return calculateC_m_dampWithCache(x, omega, v);
    }

    // shape stuff
    double AeroComponent::referenceArea(){
        return shape()->referenceArea();
    }

    double AeroComponent::wettedArea(){
        return shape()->wettedArea();
    }
    
    double AeroComponent::planformArea(){
        return shape()->planformArea();
    }

    Eigen::Vector3d AeroComponent::planformCenter(){
        return shape()->planformCenter();
    }

    // getter and setter for shape
    Shapes::AeroComponentShape* AeroComponent::shape(){
        return _shape.get();
    }

    void AeroComponent::setShape( std::unique_ptr<Shapes::AeroComponentShape> shape ){
        _shape = std::move(shape);
        clearCaches();
    }

    // getter and setter for finish
    Finish* AeroComponent::finish(){
        return _finish.get();
    }
    
    void AeroComponent::setFinish( std::unique_ptr<Finish> finish ){
        _finish = std::move( finish );
        clearCaches(); // only need to clear drag
    }
    
    // getter and setter for material
    Material* AeroComponent::material(){
        return _material.get();
    }

    void AeroComponent::setMaterial( std::unique_ptr<Material> material ){
        _material = std::move(material);
        clearCaches(); // only need to clear non-aero caches   
    }

    double AeroComponent::calculateMass(double time){
        return shape()->volume() * material()->density;
    }

    Eigen::Matrix3d AeroComponent::calculateInertia(double time){
        // inertia about cm
        auto thisInertia = shape()->inertia(); // inertia/density of the shape about cm
        auto disp = -calculateCm(time); // moving the moment of inertia back towards the origin
        auto thisVolume = shape()->volume();
        auto zeroInertia = Utils::parallel_axis_transform(thisInertia, disp, thisVolume);
        zeroInertia *= material()->density; // multiplying by density so it's the actual inertia
        return zeroInertia;
    }

    Eigen::Vector3d AeroComponent::calculateCm(double time){
        auto thisCm = shape()->cm(); // cm relative to position
        auto bodyCm = position() + thisCm;
        return bodyCm;
    }

    AeroComponent* AeroComponent::parent(){
        if(_parent.expired()) return NULL;
        if(_parent.lock().get() == NULL) return NULL;
        return _parent.lock().get();
    }

    // need to override this so its operating on the correct parent
    void AeroComponent::setParent( Component* parent ) {
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