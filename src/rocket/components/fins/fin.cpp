#include "fin.hpp"
#include <cmath>
#include "maths.hpp"

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

    Eigen::Array<double, 6, 1> Fin::cpInterpPolyCoeffs() const {
        // adapted this to calculate these
        // https://github.com/openrocket/openrocket/blob/unstable/core/src/net/sf/openrocket/aerodynamics/barrowman/FinSetCalc.java#L581
        // index is the power of x it's the coeff for
        if(!_cpInterpPolyCoeffs.hasNaN()) return _cpInterpPolyCoeffs;
        Eigen::Array<double, 6, 1> coeffs;
        const auto ar = 2*yMax()/planformArea();
        const double denom = std::pow(1 - 3.4641 * ar, 2); // common denominator
		coeffs[5] = (-1.58025 * (-0.728769 + ar) * (-0.192105 + ar)) / denom;
		coeffs[4] = (12.8395 * (-0.725688 + ar) * (-0.19292 + ar)) / denom;
		coeffs[3] = (-39.5062 * (-0.72074 + ar) * (-0.194245 + ar)) / denom;
		coeffs[2] = (55.3086 * (-0.711482 + ar) * (-0.196772 + ar)) / denom;
		coeffs[1] = (-31.6049 * (-0.705375 + ar) * (-0.198476 + ar)) / denom;
		coeffs[0] = (9.16049 * (-0.588838 + ar) * (-0.20624 + ar)) / denom;
        return coeffs;
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
        return planformArea()/referenceArea()*(K1+K2*alpha+K3*std::pow(alpha,2));
    }

    // cna for below 0.5
    double Fin::calculateC_n_a( double mach, double alpha, double gamma) const {
        if(mach <= 1) return subsonicCNa(mach, alpha);
        return supersonicCNa(mach, alpha, gamma);
    }

    Eigen::Vector3d Fin::calculateCp( double mach, double alpha, double gamma) const {
        double CPx;
        const double AR =  2*yMax()/planformArea();
        const double b = Utils::beta(mach);
        if(mach <= 0.5){
            CPx = xMacLeadingEdge() + mac()/4;
        } else if(mach < 2){
            const auto coeffs = cpInterpPolyCoeffs();
            const Eigen::Array<double, 6, 1> vals = (Eigen::Array<double, 6, 1>::Ones()*mach).pow(Eigen::Array<double, 6, 1>{0,1,2,3,4,5});
            CPx = (coeffs*vals).sum();
        } else {
            CPx = (AR*b - 0.67)/(2*AR*b-1)*mac();
        }

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
        _cpInterpPolyCoeffs = cpInterpPolyCoeffs();
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
}