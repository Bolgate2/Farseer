#pragma once
#include <memory>
#include <map>
#include <concepts>

namespace Rocket{

class AbstractAerodynamicCalculator{
    public:
        virtual double CnAlpha(double mach, double alpha) = 0;
};

template<class T>
class AerodynamicCalculator : public AbstractAerodynamicCalculator{
    private:
        T* component;
    protected:
        T* getComponent(){ return component; }
    public:
        template<class C>
        AerodynamicCalculator(C* comp){
            component = static_cast<T*>(comp);
        }

        double CnAlpha(double mach, double alpha);
};

template<class T> concept IsAeroCalc = std::derived_from<T,AbstractAerodynamicCalculator>;

}