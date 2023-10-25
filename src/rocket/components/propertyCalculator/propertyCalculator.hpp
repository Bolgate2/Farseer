# ifndef COMPONENT_PROPERTY_CALCULATOR_H_
# define COMPONENT_PROPERTY_CALCULATOR_H_

#include <memory>

namespace Rocket{

    #ifndef COMPONENT_H_
    class Component; // forward declaring component if it hasn't been declared already
    #endif
    template<typename T, typename... Ps>
    class PropertyCalculator{
        private:
            //std::weak_ptr<Component> _component;
        public:
            T calculate(Ps... args);
    };

}

#endif