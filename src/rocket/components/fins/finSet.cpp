#include "finSet.hpp"
#include <cmath>
#include <vector>
#include "maths.hpp"

namespace Rocket{
    std::string FinSet::defaultName = "Fin Set";

    // constructor
    FinSet::FinSet(int numFins, std::string name, Eigen::Vector3d position):
    ExternalComponent(nullptr, nullptr, nullptr, name, position){
        setNumFins(numFins);
    }

    // creator
    std::shared_ptr<FinSet> FinSet::create(std::unique_ptr<Fin> fin, int numFins, AeroComponent* parent, std::string name, Eigen::Vector3d position){
        auto obj = std::shared_ptr<FinSet>(
            new FinSet(numFins, name, position)
            );
        if(parent != nullptr){
            parent->addComponent(obj.get());
        }
        obj->setFin(std::move(fin));
        return obj;
    }


    void FinSet::clearCaches(){
        ExternalComponent::clearCaches();
        _finRotations = {};
    }

    Finish* FinSet::finish() const {
        return fin()->finish();
    }

    void FinSet::setFinish( std::unique_ptr<Finish> finish ){
        return fin()->setFinish(std::move(finish));
    }

    Material* FinSet::material() const {
        return fin()->material();
    }

    void FinSet::setMaterial( std::unique_ptr<Material> material ){
        return fin()->setMaterial(std::move(material));
    }

    double FinSet::calculateMass(double time) const {
        return fin()->mass(time)*numFins();
    }

    std::vector<Eigen::Matrix3d> FinSet::finRotations() const {
        if(!_finRotations.empty()) return _finRotations;
        std::vector<Eigen::Matrix3d> rotations = {};
        auto spacing = (2*M_PI)/numFins();
        for(int i = 0; i < numFins(); i++){
            Eigen::Matrix3d quat = Eigen::AngleAxis<double>(i*spacing, Eigen::Vector3d(1,0,0)).toRotationMatrix();
            rotations.push_back(quat);
        }
        return rotations;
    }

    Eigen::Matrix3d FinSet::calculateInertia(double time) const {
        auto finLocalInertia = fin()->inertia(time);
        auto finLocalCm = fin()->cm(time);
        auto finMass = fin()->mass(time);
        const Eigen::Vector3d pos = position();
        auto rotations = finRotations();
        Eigen::Matrix3d totalInertia = Eigen::Matrix3d::Zero();
        //std::cout << "FIN MASS " << finMass << std::endl; // correct
        //std::cout << "FIN CM [" << finLocalCm.transpose() << "]" << "\n"; // correct
        //std::cout << "FIN SET RAD" << bodyRadius( position().x() ) << std::endl; // correct
        Eigen::Vector3d finDisp = finLocalCm + Eigen::Vector3d{ 0, bodyRadius(position().x()), 0 }; // distance from the rockets centerline to the fins cm

        for(auto rotPtr = rotations.cbegin(); rotPtr != rotations.cend(); rotPtr++){
            auto rot = *rotPtr;
            // rotating the fins dispacement vector
            Eigen::Vector3d thisFinDisp = rot*finDisp;
            thisFinDisp = (std::numeric_limits<double>::epsilon() < thisFinDisp.array().abs()).select(thisFinDisp, 0);
            // rotating the fins inertia about its own cm, this is ok at the displacement vector is to the fins cm
            Eigen::Matrix3d finRotatedInertia = (rot*finLocalInertia)*rot.transpose();
            Eigen::Vector3d thisFinTotalDisp = pos+thisFinDisp;
            
            Eigen::Matrix3d finGlobalInertia = Utils::parallel_axis_transform(finRotatedInertia, thisFinTotalDisp, finMass);
            totalInertia += finGlobalInertia;
        }
        // rounding inertia
        totalInertia = (std::numeric_limits<double>::epsilon() < totalInertia.array().abs()).select(totalInertia, 0);
        return totalInertia;
    }

    Eigen::Vector3d FinSet::calculateCm(double time) const {
        // assuming that the fin sets CM rests on the centerline
        Eigen::Vector3d pos = position();
        auto finLocalCm = fin()->cm(time);
        return pos + Eigen::Vector3d{finLocalCm.x(), 0, 0};
    }

    double FinSet::calculateC_n_a( double mach, double alpha, double gamma) const {
        auto N = numFins();
        auto finCna = fin()->c_n_a(mach, alpha, gamma);
        // multiple fin effects
        double coeff;
        if(N < 5){
            coeff = 1;
        } else if (N > 8){
            coeff = 0.75;
        } else {
            switch(N){
                case 5:
                    coeff = 0.948;
                    break;
                case 6:
                    coeff = 0.913;
                    break;
                case 7:
                    coeff = 0.854;
                    break;
                default:
                    coeff = 1;
            }
        }
        finCna *= coeff * N/2; // N/2 is only valid for 3+ fins
        // fin body interference
        auto finSpan = fin()->yMax();
        auto br = bodyRadius(position().x());
        double finBodyInterference;
        if(finSpan+br <= 0){
            finBodyInterference = 0;
        } else {
            finBodyInterference = 1 + br/(br+finSpan);
        }
        finCna *= finBodyInterference;
        return finCna;
    }

    double FinSet::calculateC_m_a( double mach, double alpha, double gamma) const {
        return 0;
    }

    Eigen::Vector3d FinSet::calculateCp( double mach, double alpha, double gamma) const {
        auto finCP = fin()->cp(mach, alpha, gamma);
        return position() + Eigen::Vector3d{ finCP.x(), 0, 0 }; //assuming that the fins cp is on the central axis
    }

    double FinSet::calculateC_m_damp(double x) const {
        auto finArea = fin()->planformArea();
        auto finDist = bodyRadius(position().x()) + fin()->yMac(); // assuming that this force is acting at the MAC
        auto numExposedFins = std::min(numFins(), 4); // ORK says max fins exposed is 4
        return 0.6*(numExposedFins*finArea*finDist)/(referenceArea()*referenceLength());
    }

    Fin* FinSet::fin() const {
        return _fin.get();
    }

    void FinSet::setFin( std::unique_ptr<Fin> fin ){
        fin->setFinSet(this);
        _fin = std::move(fin);
        clearCaches();
    }

    int FinSet::numFins() const {
        return _numFins;
    }

    void FinSet::setNumFins( int num ){
        _numFins = num;
        clearCaches();
        _finRotations = finRotations();
    }

    double FinSet::wettedArea() const {
        return fin()->wettedArea()*numFins();
    }

    double FinSet::calculateLowestPoint() const {
        return position().x() + fin()->calculateLowestPoint();
    }

    double FinSet::maxSurfaceDistanceTravelled() const {
        return fin()->maxSurfaceDistanceTravelled();
    }
    
    double FinSet::calculateSurfaceDistanceTravelled(double x) const {
        if( x < position().x()) return 0;
        if( x >= calculateLowestPoint() ) return maxSurfaceDistanceTravelled();
        return x - position().x();
    }
}