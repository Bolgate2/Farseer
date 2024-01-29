#pragma once
#include <memory>
#include <map>

namespace Rocket{

template<class T>
class AerodynamicCalculator{
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


//template<class T> concept IsAeroCalc = std::is_base_of<AbstractAerodynamicCalculator, T>::value && std::is_class<T>::value;

}