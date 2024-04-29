#pragma once

#include <mutex>
#include <map>

namespace RealAtmos
{
    const extern double R_0;
    
    struct MOLE
    {
        double n;
        double M;
    };

    class RealAtmos
    {
        private:
            static RealAtmos * pinstance_;
            static std::mutex mutex_;

        
        protected:
            RealAtmos();
            ~RealAtmos();
            
            std::map<double, MOLE> nMap;

            double H_(double z);
            double Tm_(double z);
            double n_(double z);
            double M_(double z);
            
            double K_(double z);
            double dTdZ_(double z);

        public:
            // constructor stuff
            RealAtmos(RealAtmos &other) = delete;

            void operator=(const RealAtmos &) = delete;

            static RealAtmos * GetInstance();

            // atmospheric property funcs
            double temperature(double z);
            double pressure(double z);
            double density(double z);
            double g(double z);
            double sound(double z);
            double dynamic_viscosity(double z);
            double kinematic_viscosity(double z);

            // constant accessors

    };

}
