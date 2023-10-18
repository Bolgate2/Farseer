# ifndef MASS_COMPONENT_H_
# define MASS_COMPONENT_H_

#include "component.hpp"

namespace Rocket{
    class MassComponent: public Component{
        private:
            double _mass;
            bool _massOverride;
        public:
            virtual double get_mass(double time);
            void overrideMass(bool);
            void setMass(double mass);
    };
}

#endif