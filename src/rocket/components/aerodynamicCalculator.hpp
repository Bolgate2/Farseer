#pragma once
#include <memory>
#include <map>

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
        AerodynamicCalculator(T* comp){
            component = comp;
        }
        std::map<double, std::map<double, double>> CnAlphaCache = std::map<double, std::map<double, double>>();
        virtual double CnAlpha(double mach, double alpha) override { return 0;}
};


template<class T> concept IsAeroCalc = std::is_base_of<AbstractAerodynamicCalculator, T>::value && std::is_class<T>::value;

}