#pragma once
#include <memory>
#include <map>

namespace Rocket{
/*
class AbstractComponent;

class KinematicCalculator{
    private:
        std::weak_ptr<AbstractComponent> component;
    public:
        // probably change this to use a sharedptr
        KinematicCalculator(AbstractComponent* comp);
        virtual double mass(double time);
};
*/
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
        KinematicCalculator(T* comp){
            component = comp;
        }
        std::map<double, std::map<double, double>> CnAlphaCache = std::map<double, std::map<double, double>>();
        virtual double mass(double time) { return 0; }
};

template<class T> concept IsKinCalc = std::is_base_of<AbstractKinematicCalculator, T>::value && std::is_class<T>::value;

}