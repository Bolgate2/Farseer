#include "fin.hpp"
#include <cmath>
#include "maths.hpp"
#include <fmt/core.h>

namespace Rocket{
    std::string Fin::defaultName = "Fin";
    // constructor
    Fin::Fin(std::unique_ptr<Shapes::FinComponentShape> shape, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, std::string name):
    ExternalComponent(nullptr, std::move(material), std::move(finish), name, Eigen::Vector3d{0,0,0})
    {
        setShape( std::move(shape) );
    }
    // aero component functions

    void Fin::clearCaches(){
        ExternalComponent::clearCaches();
        _cpInterpPolyCoeffs = Eigen::Array<double, 6, 1>::Ones()*NAN_D;
    }

    FinSet* Fin::finSet() const {
        if(_finSet.expired()) return nullptr;
        if(_finSet.lock().get() == nullptr) return nullptr;
        return _finSet.lock().get();
    }
    
    void Fin::setFinSet( FinSet* finSet ){
        _finSet = std::dynamic_pointer_cast<FinSet>(finSet->shared_from_this());
    }

    double Fin::calculateCpInterpPoly(const double mach, const double ar) const {
        static const Eigen::Matrix<double, 6, 3 > numerCoeffs {
            {9.7900515337,             -7.7838569163,              1.1889239133},
            {-33.7769972053,              30.529368226,             -4.7287796087},
            {59.1097451092,            -53.6866891547,              8.2753643153},
            {-42.2212465066,             38.6317948476,             -5.9109745109},
            {13.7219051146,            -12.6050612795,              1.9210667161},
            {-1.6888498603,              1.5552173847,             -0.2364389804}
        };
        static const Eigen::Vector3d denomCoeffs {12.8247036264, -7.404346091, 1.0687253022};
        static const Eigen::Array3d pow2arr {2,1,0};
        static const Eigen::Array<double, 1,6> pow5arr {0, 1, 2, 3, 4, 5};
        Eigen::Vector3d arVec = (Eigen::Array3d::Ones()*ar).pow(pow2arr);
        Eigen::Vector<double,6> machVec = (Eigen::Array<double,1,6>::Ones()*mach).pow(pow5arr);
        double cpVal = (numerCoeffs*arVec).dot(machVec)/denomCoeffs.dot(arVec);
        return cpVal;
    }


    double Fin::subsonicCNa( double mach, double alpha ) const {
        const auto h2 = std::pow(yMax(), 2);
        const auto num = 2*M_PI*h2/referenceArea();
        auto chordSweep = midChordSweep();
        const auto sqrtPart = 1 + std::pow( Utils::beta(mach)*h2/( planformArea()*std::cos( chordSweep ) ),2 ) ;
        auto CNa1 = num/( 1+ std::sqrt(sqrtPart) );
        return CNa1;
    }

    double Fin::supersonicCNa( double mach, double alpha, double gamma) const {
        // TODO: precompute a whole bunch of K values
        const auto b = Utils::beta(mach);
        const auto K1 = 2/b;
        const auto K2 = ( (gamma+1)*std::pow(mach,4) - 4*std::pow(b,2) )/(4*std::pow(b,4));
        const auto K3 = ( (gamma+1)*std::pow(mach,8) + (2*std::pow(gamma,2) - 7*gamma - 5)*std::pow(mach,6) + 10*(gamma+1)*std::pow(mach,4) + 8 )/(6*std::pow(b,7));
        // ORK eq 3.49
        auto cna = planformArea()/referenceArea()*(K1+K2*alpha+K3*std::pow(alpha,2));
        return cna;
    }

    double Fin::transonicCNa( double mach, double alpha, double gamma ) const {
        // finding derivatives like this bc I'm tired
        const auto cd09 = subsonicCNa(0.9, alpha);
        const auto dcd09 = (subsonicCNa(0.90001, alpha) - cd09)/0.00001;
        const auto cd15 = supersonicCNa(1.5, alpha, gamma);
        const auto dcd15 = (supersonicCNa(1.50001, alpha, gamma) - cd15)/0.00001;

        static const Eigen::Matrix<double,5,4> coeffScales {
            {625.0/27, 250.0/27, -625.0/27, 125.0/27 }, //corr
            {-2750.0/27, -125.0/3, 2750.0/27, -175.0/9 }, //corr
            {325.0/2, 135.0/2, -325.0/2, 30 }, //corr
            {-225.0/2, -185.0/4, 225.0/2, -81.0/4}, //corr
            {475.0/16, 45.0/4, -459.0/16, 81.0/16}
        };
        Eigen::Vector<double, 4> coeffs { cd09, dcd09, cd15, dcd15 };
        // a, b, c, d, e
        Eigen::Vector<double,5> machVec {
            std::pow(mach,4),
            std::pow(mach,3),
            std::pow(mach,2),
            mach,
            1
        };
        double cna = (coeffScales*coeffs).dot(machVec);
        return cna;
    }

    double Fin::calculateC_n_a( double mach, double alpha, double gamma) const {
        double cna;
        if(mach <= 0.9)
        {
            cna = subsonicCNa(mach, alpha);
        }
        else if(mach <= 1.5)
        {
            cna = transonicCNa(mach, alpha, gamma);
        }
        else
        {
            cna = supersonicCNa(mach, alpha, gamma);
        }
        
        return cna;
    }

    Eigen::Vector3d Fin::calculateCp( double mach, double alpha, double gamma) const {
        double CPx;
        const double AR =  2*std::pow(yMax(),2)/planformArea();
        const double b = Utils::beta(mach);
        if(mach <= 0.5){
            CPx = 0.25;
        } else if(mach < 2){
            CPx = calculateCpInterpPoly(mach, AR);
        } else {
            CPx = (AR*b - 0.67)/(2*AR*b-1);
        }
        CPx *= mac();
        CPx += xMacLeadingEdge();
        Eigen::Vector3d vec { CPx, yMac(), 0 };
        return vec;
    }

    // shape stuff
    Shapes::FinComponentShape* Fin::shape() const {
        return _shape.get();
    }

    void Fin::setShape( std::unique_ptr<Shapes::AeroComponentShape> shape ){
        // try to cast the underlying pointer to a pointer of the inherited class
        auto castedShapePointer = dynamic_cast<Shapes::FinComponentShape*>(shape.get());

        if(castedShapePointer != nullptr){
            // if casting is successful the a unique pointer of the correct type is crated and the old one is released
            auto newShapeUniquePtr = std::unique_ptr<Shapes::FinComponentShape>(castedShapePointer);
            shape.release();
            setShape(std::move(newShapeUniquePtr));
        }
        // if the function has reached here it has failed
        std::cerr << "Invalid shape for body tube\n"; // TODO: make this more descriptive
    }

    void Fin::setShape( std::unique_ptr<Shapes::ExternalComponentShape> shape ){
        // try to cast the underlying pointer to a pointer of the inherited class
        auto castedShapePointer = dynamic_cast<Shapes::FinComponentShape*>(shape.get());

        if(castedShapePointer != nullptr){
            // if casting is successful the a unique pointer of the correct type is crated and the old one is released
            auto newShapeUniquePtr = std::unique_ptr<Shapes::FinComponentShape>(castedShapePointer);
            shape.release();
            setShape(std::move(newShapeUniquePtr));
        }
        // if the function has reached here it has failed
        std::cerr << "Invalid shape for body tube\n"; // TODO: make this more descriptive
    }

    void Fin::setShape( std::unique_ptr<Shapes::FinComponentShape> shape ){
        _shape = std::move(shape);
        clearCaches();
    }

    Eigen::Vector3d Fin::calculateCm(double time) const {
        return shape()->planformCenter();
    }

    Eigen::Matrix3d Fin::calculateInertia(double time) const {
        // returns inertia about cm not about the origin
        return shape()->inertia() * material()->density;
    }
    
    Eigen::Vector3d Fin::position() const {
        return finSet()->position();
    }

    double Fin::referenceArea() const {
        return finSet()->referenceArea();
    } // using reference area of the parent

    double Fin::referenceLength() const {
        return finSet()->referenceLength();
    } // using reference area of the parent

    double Fin::lowestPoint() const {
        return finSet()->lowestPoint();
    }

    double Fin::calculateLowestPoint() const {
        return chord(0);
    }

    double Fin::maxSurfaceDistanceTravelled() const {
        return chord(0);
    }
    
    double Fin::calculateSurfaceDistanceTravelled(double x) const {
        if( x < position().x()) return 0;
        if( x >= calculateLowestPoint() ) return maxSurfaceDistanceTravelled();
        return x;
    }

    double Fin::surfaceDistanceTravelled(double x) const {
        return finSet()->surfaceDistanceTravelled(x);
    }

    double Fin::calculateCdfA(const double mach, const double reL) const {
        double cf = Cf(mach, reL);
        double cdfa = (1 + 2*thickness()/mac())*wettedArea()*cf/referenceArea();
        return cdfa;
    }

    double Fin::cDotStag(const double mach) const {
        double qstagonq; 
        if(mach <= 1){
            qstagonq = 1 + std::pow(mach,2.0)/4 + std::pow(mach,4.0)/40;
        } else {
            qstagonq = 1.84 - 0.76/std::pow(mach,2) + 0.166/std::pow(mach,4) + 0.035/std::pow(mach,6);
        }
        return 0.85*qstagonq; // should never reach here, just here so that compiler stops complaining
    }

    double Fin::calculateCdpA(const double mach) const {
        auto frontCdpdot = cDotStag(mach)*0.5; // 0.5 is placeholder, should be cos(Gamma_l)^2
        auto rearCdpdot = Cdotb(mach);
        auto cdpdot = frontCdpdot + rearCdpdot;
        auto refArea = thickness()*yMax()/referenceArea();
        return cdpdot*refArea;
    }
}