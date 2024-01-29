#pragma once
#include <memory>
#include <map>

namespace Rocket{

class AbstractComponent;

class AerodynamicCalculator{
    private:
        std::weak_ptr<AbstractComponent> component;
    public:
        AerodynamicCalculator(AbstractComponent* comp);
        std::map<double, std::map<double, double>> CnAlphaCache = std::map<double, std::map<double, double>>();
        virtual double CnAlpha(double mach, double alpha);
};


template<class T> concept IsAeroCalc = std::is_base_of<AerodynamicCalculator, T>::value && std::is_class<T>::value;

}