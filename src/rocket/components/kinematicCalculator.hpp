#pragma once
#include <memory>

namespace Rocket{

class AbstractComponent;

class KinematicCalculator{
    private:
        std::weak_ptr<AbstractComponent> component;
    public:
        // probably change this to use a sharedptr
        KinematicCalculator(AbstractComponent* comp);
        virtual double mass(double time);
};

template<class T> concept IsKinCalc = std::is_base_of<KinematicCalculator, T>::value && std::is_class<T>::value;

}