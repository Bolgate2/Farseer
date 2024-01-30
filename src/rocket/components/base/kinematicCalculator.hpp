#pragma once
#include <memory>
#include <map>

namespace Rocket{

class AbstractKinematicCalculator{
    public:
        virtual double mass(double time) = 0;
};

template<class T>
class KinematicCalculator : public AbstractKinematicCalculator{
    private:
        T* component;
    protected:
        T* getComponent(){ return component; }
    public:
        template<class C>
        KinematicCalculator(C* comp){
            component = static_cast<T*>(comp);
        }

        std::map<double, std::map<double, double>> CnAlphaCache = std::map<double, std::map<double, double>>();
        double mass(double time);
};

template<class T> concept IsKinCalc = std::derived_from<T,AbstractKinematicCalculator>;
}