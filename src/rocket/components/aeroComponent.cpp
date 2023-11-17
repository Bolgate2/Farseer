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

    AeroComponent* AeroComponent::parent() const {
        if(_parent.expired()) return NULL;
        if(_parent.lock().get() == NULL) return NULL;
        return _parent.lock().get();
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
        _c_m_dampCache.clear();
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

    // checking that args exist in cache
    bool AeroComponent::c_n_aExists(double mach, double alpha, double gamma) const {
        auto alphaCacheKey = _c_n_aCache.find(mach);
        if(alphaCacheKey != _c_n_aCache.end()){
            auto alphaCache = alphaCacheKey->second;
            auto gammaCacheKey = alphaCache.find(alpha);
            if(gammaCacheKey != alphaCache.end()){
                auto gammaCache = gammaCacheKey->second;
                auto valKey = gammaCache.find(gamma);
                if(valKey != gammaCache.end()){
                    return true;
                }
            }
        }
        return false;
    }

    bool AeroComponent::c_m_aExists(double mach, double alpha, double gamma) const {
        auto alphaCacheKey = _c_m_aCache.find(mach);
        if(alphaCacheKey != _c_m_aCache.end()){
            auto alphaCache = alphaCacheKey->second;
            auto gammaCacheKey = alphaCache.find(alpha);
            if(gammaCacheKey != alphaCache.end()){
                auto gammaCache = gammaCacheKey->second;
                auto valKey = gammaCache.find(gamma);
                if(valKey != gammaCache.end()){
                    return true;
                }
            }
        }
        return false;
    }

    bool AeroComponent::cpExists(double mach, double alpha, double gamma) const {
        auto alphaCacheKey = _cpCache.find(mach);
        if(alphaCacheKey != _cpCache.end()){
            auto alphaCache = alphaCacheKey->second;
            auto gammaCacheKey = alphaCache.find(alpha);
            if(gammaCacheKey != alphaCache.end()){
                auto gammaCache = gammaCacheKey->second;
                auto valKey = gammaCache.find(gamma);
                if(valKey != gammaCache.end()){
                    return true;
                }
            }
        }
        return false;
    }

    // component and cache calculations of elems
    //C_n
    double AeroComponent::calculateC_n_aWithComponents( double mach, double alpha, double gamma) const {
        auto thisC_n_a = calculateC_n_a(mach, alpha, gamma);
        auto aeroComps = aeroComponents();
        for(auto aeroComp = aeroComps.begin(); aeroComp != aeroComps.end(); ++aeroComp){
            auto compC_n_a = (*aeroComp)->c_n_a(mach, alpha, gamma);
            auto compAref = (*aeroComp)->referenceArea();
            auto compAdjC_n_a = compC_n_a*compAref/referenceArea();
            thisC_n_a += compAdjC_n_a;
        }
        return thisC_n_a;
    }

    double AeroComponent::c_n_a( double mach, double alpha, double gamma) const {
        if(!caching()) return calculateC_n_aWithComponents(mach, alpha, gamma);
        // checking cache
        if(c_n_aExists(mach, alpha, gamma)) return _c_n_aCache.find(mach)->second.find(alpha)->second.find(gamma)->second;
        // calculating if not there
        auto CNa = calculateC_n_aWithComponents(mach, alpha, gamma);
        //_c_n_aCache[mach][alpha][gamma] = CNa;
        return CNa;
    }

    double AeroComponent::c_n( double mach, double alpha, double gamma) const {
        return c_n_a(mach, alpha, gamma)*alpha;
    }

    //C_m
    double AeroComponent::calculateC_m_aAtOrigin( double mach, double alpha, double gamma) const {
        // using ORK eq 3.3, dividing both sides by alpha
        /* just using the components coefficients rather than their children as only this comp
         * is relevant here
         */
        auto thisC_m_a = calculateC_m_a(mach, alpha, gamma);
        auto thisC_n_a = calculateC_n_a(mach, alpha, gamma);
        auto dist = cp(mach, alpha, gamma).x(); // because this is a moment about the y axis caused by forces in the z direction, only the x distance matters, CNa is acting about CP
        auto newC_m_a = (thisC_m_a*referenceLength() - thisC_n_a*dist) / referenceLength();
        return newC_m_a;
    }

    double AeroComponent::calculateC_m_aWithComponents( double mach, double alpha, double gamma) const {
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

    double AeroComponent::c_m_a( double mach, double alpha, double gamma) const {
        if(!caching()) return calculateC_m_aWithComponents(mach, alpha, gamma);
        //if(c_m_aExists(mach, alpha, gamma)) return _c_m_aCache[mach][alpha][gamma];
        if(c_m_aExists(mach, alpha, gamma)) return _c_m_aCache.find(mach)->second.find(alpha)->second.find(gamma)->second;
        // do interpolation here
        return calculateC_m_aWithComponents(mach, alpha, gamma);
    }

    double AeroComponent::c_m( double mach, double alpha, double gamma) const {
        return c_m_a(mach, alpha, gamma) * alpha;
    }

    //CP
    Eigen::Vector3d AeroComponent::calculateCpWithComponents( double mach, double alpha, double gamma) const {
        // getting cp using ORK eq 3.29
        // because this only considers C_n_a it will only affect the x position of cp, can be improved in future
        Eigen::Vector3d weightedCpSum = Eigen::Vector3d::Zero();
        double c_n_aSum = 0.0;
        auto aeroComps = aeroComponents();
        for(auto aeroComp = aeroComps.begin(); aeroComp != aeroComps.end(); ++aeroComp){
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

    Eigen::Vector3d AeroComponent::cp( double mach, double alpha, double gamma) const {
        if(!caching()) return calculateCpWithComponents(mach, alpha, gamma);
        //if(cpExists(mach, alpha, gamma)) return _cpCache[mach][alpha][gamma];
        if(cpExists(mach, alpha, gamma)) return _cpCache.find(mach)->second.find(alpha)->second.find(gamma)->second;
        return calculateCpWithComponents(mach, alpha, gamma);
    }

    //C_m_damp
    double AeroComponent::calculateC_m_dampWithComponents(double x) const {
        auto thisC_m_damp = calculateC_m_damp(x);
        auto aeroComps = aeroComponents();
        for(auto aeroComp = aeroComps.begin(); aeroComp != aeroComps.end(); ++aeroComp){
            auto compC_m_damp = (*aeroComp)->c_m_damp(x, 1, 1); // using 1 here so that omega^2/v^2 has no effect
            auto compAref = (*aeroComp)->referenceArea();
            auto compDref = (*aeroComp)->referenceLength();
            auto compAdjC_m_damp = compC_m_damp* (compAref*compDref) / (referenceArea()*referenceLength());
            thisC_m_damp += compAdjC_m_damp;
        }
        return thisC_m_damp;
    }

    double AeroComponent::c_m_damp(double x, double omega, double v) const {
        if(omega == 0 || v == 0) return 0;
        if(!caching()) return calculateC_m_dampWithComponents(x)*std::pow(omega,2)/std::pow(v,2);
        auto cacheKey = _c_m_dampCache.find(x);
        if(cacheKey != _c_m_dampCache.end()) return cacheKey->second*std::pow(omega,2)/std::pow(v,2);
        auto cmDampNoVel = calculateC_m_dampWithComponents(x);
        _c_m_dampCache[x] = cmDampNoVel;
        return cmDampNoVel*std::pow(omega,2)/std::pow(v,2);
    }

    // shape stuff
    double AeroComponent::referenceArea() const {
        return shape()->referenceArea();
    }

    double AeroComponent::wettedArea() const {
        return shape()->wettedArea();
    }
    
    double AeroComponent::planformArea() const {
        return shape()->planformArea();
    }

    Eigen::Vector3d AeroComponent::planformCenter() const {
        return shape()->planformCenter();
    }

    // getter and setter for shape
    Shapes::AeroComponentShape* AeroComponent::shape() const {
        return _shape.get();
    }

    void AeroComponent::setShape( std::unique_ptr<Shapes::AeroComponentShape> shape ){
        _shape = std::move(shape);
        clearCaches();
    }

    // getter and setter for finish
    Finish* AeroComponent::finish() const {
        return _finish.get();
    }
    
    void AeroComponent::setFinish( std::unique_ptr<Finish> finish ){
        _finish = std::move( finish );
        clearCaches(); // only need to clear drag
    }
    
    // getter and setter for material
    Material* AeroComponent::material() const {
        return _material.get();
    }

    void AeroComponent::setMaterial( std::unique_ptr<Material> material ){
        _material = std::move(material);
        clearCaches(); // only need to clear non-aero caches   
    }

    double AeroComponent::calculateMass(double time) const {
        return shape()->volume() * material()->density;
    }

    Eigen::Matrix3d AeroComponent::calculateInertia(double time) const {
        // inertia about cm
        Eigen::Matrix3d thisInertia = shape()->inertia() * material()->density; // inertia/density of the shape about cm
        auto disp = calculateCm(time); // moving the moment of inertia back towards the origin
        auto zeroInertia = Utils::parallel_axis_transform(thisInertia, disp, calculateMass(time));
        return zeroInertia;
    }

    Eigen::Vector3d AeroComponent::calculateCm(double time) const {
        auto thisCm = shape()->cm(); // cm relative to position
        Eigen::Vector3d bodyCm = position() + thisCm;
        return bodyCm;
    }

    double AeroComponent::Cda2Cd(const double Cda, const double alpha) const {
        double cd;
        double adjFac;
        double absAlpha = std::abs(alpha);
        if(absAlpha <= 17*M_PI/180){
            adjFac = -22.9706*std::pow(absAlpha,3) + 10.2233*std::pow(absAlpha,2) + 1;
        } else {
            adjFac = -1.48*std::pow(absAlpha,4) + 6.7849*std::pow(absAlpha,3) - 10.0627*std::pow(absAlpha,2) + 4.334*absAlpha + 0.7342;
        }
        cd = adjFac*Cda;
        return cd;
    }

    double AeroComponent::lowestPoint() const {
        if(parent() == nullptr) return calculateLowestPoint();
        return parent()->lowestPoint();
    }

    double AeroComponent::surfaceDistanceTravelled(double x) const {
        if(parent() == nullptr) return calculateSurfaceDistanceTravelled(x);
        return parent()->surfaceDistanceTravelled(x);
    }

    double AeroComponent::CfOrk(const double mach, const double reL) const {
        //const double rockL = lowestPoint();
        const double rockL = surfaceDistanceTravelled(lowestPoint());
        double Re = reL*rockL;

        double cf;
        if(Re < 1e4){
            cf = 1.48*1e-2;
            cf *= 1.0-0.1*std::pow(mach,2);
        } else {
            double turbCf = 1/std::pow(1.5*std::log(Re) - 5.6, 2);
            double matCf = 0.032*std::pow(finish()->roughness/rockL, 0.2);

            if(mach <= 1){
                turbCf *= 1.0-0.1*std::pow(mach,2);
                matCf *= 1.0-0.1*std::pow(mach,2);
                cf = std::max(turbCf, matCf);
            } else {
                turbCf /= std::pow(1+0.15*std::pow(mach,2),0.58);
                matCf /= 1 + 0.18*std::pow(mach,2);
                cf = std::max(turbCf, matCf);
            }
        }
        return cf;
    }

    double AeroComponent::CfNew(const double mach, const double reL) const {
        if(reL <= 0) return 0;

        double lamCritDist = 1e4/reL;
        double turbCritDist = 51*std::pow(finish()->roughness/lowestPoint(), -1.039)/reL;

        double topPoint = surfaceDistanceTravelled(position().x());
        double bottomPoint = surfaceDistanceTravelled(calculateLowestPoint());
        double maxDist = bottomPoint - maxDist;

        // distances of each region on the component
        double lamDist = std::clamp(lamCritDist-topPoint, 0.0, maxDist);
        double turbDist = std::clamp(turbCritDist-(topPoint+lamDist), 0.0, maxDist-lamDist);
        double matDist = std::max(maxDist-(turbDist+lamDist), 0.0);

        // turbulence start in global coords
        double turbStart = lamCritDist;
        double turbEnd = turbStart + turbDist;

        // getting friction coeffs (not adjusted for compressibility)
        double lamCf = 1.48*1e-2;
        double turbCf;
        // numerically integrate for average turbulant Cf
        if(turbDist == 0){
            turbCf = 0;
        } else {
            // split into 10 regions then take average
            Eigen::Array<double, 1, 10> turbVals;
            for(int i = 0; i < 10; i++){
                double regionStart = turbStart + turbDist/10*i;
                double regionEnd = turbStart + turbDist/10*(i+1);
                double thisRe = reL*(regionStart + regionEnd)/2; // average reynolds number in the region
                turbVals[i] = 1/std::pow(1.5*std::log(thisRe) - 5.6, 2);
            }
            turbCf = turbVals.mean();
        }
        double matCf = 0.032*std::pow(finish()->roughness/lowestPoint(), 0.2);

        // adjusting for compressibility
        // no adj for lam

        if(mach <= 1){
            turbCf *= 1.0-0.1*std::pow(mach,2);
            matCf *= 1.0-0.1*std::pow(mach,2);
        } else {
            turbCf /= std::pow(1+0.15*std::pow(mach,2),0.58);
            matCf /= 1.0-0.18*std::pow(mach,2);
        }

        // taking weighted average
        // this isn't 100% accurate as the average weighting should be done on Awet exposed to each region, not length
        double avgCf = (lamDist*lamCf + turbDist*turbCf + matDist*matCf)/(lamDist + turbDist + matDist);
        return avgCf;
    }

    double AeroComponent::calculateCdfAWithComponents(const double mach, const double reL) const {
        auto thisCdf = calculateCdfA(mach, reL);
        auto thisAref = referenceArea();
        auto comps = aeroComponents();
        auto CdfSum = thisCdf;
        for(auto comp = comps.begin(); comp != comps.end(); comp++){
            auto compCdf = (*comp)->CdfA(mach, reL);
            auto compAref = (*comp)->referenceArea();
            auto compCdfAdj = compCdf*compAref/thisAref;
            CdfSum += compCdfAdj;
        }
        return CdfSum;
    }

    double AeroComponent::CdfA(const double mach, const double reL) const {
        return calculateCdfAWithComponents(mach, reL);
    }

    double AeroComponent::finenessRatioRocket() const {
        if(parent() != nullptr) return parent()->finenessRatioRocket();
        return finenessRatio();
    }

    double AeroComponent::finenessRatio() const {
        return 1;
    }
}