#pragma once
#include <memory>
#include <map>

namespace Rocket{

template<class T>
class KinematicCalculator{
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

//template<class T> concept IsKinCalc = std::is_base_of<AbstractKinematicCalculator, T>::value && std::is_class<T>::value;

}