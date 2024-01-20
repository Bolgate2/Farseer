#ifndef AERO_COMPONENT_H
#define AERO_COMPONENT_H
#include "component.hpp"
#include "rocketInterface.hpp"
#include <memory>
#include <map>

#include <Eigen/Dense>


#include "nanValues.hpp"
#include "../shapes/components/aeroComponentShape.hpp"
#include "../misc/material.hpp"
#include "../misc/finish.hpp"

// if an element exists that is this close to one higher and one lower entry of these limits, the value is interpolated instead of being calculated
#define MACH_RESOLUTION 0.01
#define ALPHA_RESOLUTION 0.1
#define GAMMA_RESOLUTION 1

namespace Rocket{

    // component with aerodynamic abilities
    class AeroComponent: public Component, public Sim::RocketInterface{
        private:
            static std::string defaultName;
            
            std::weak_ptr<AeroComponent> _parent;
            std::unique_ptr<Shapes::AeroComponentShape> _shape;

            std::unique_ptr<Finish> _finish;
            std::unique_ptr<Material> _material;
        protected:
            // inherited from component
            virtual void clearCaches() override;
            
            virtual double calculateMass(double time) const override;
            virtual Eigen::Matrix3d calculateInertia(double time) const override;
            virtual Eigen::Vector3d calculateCm(double time) const override;

            //aero functions
            //C_n_a
            virtual double calculateC_n_a( double mach, double alpha, double gamma = 1.4 ) const = 0; // VIRTUAL

            virtual double calculateC_n_aWithComponents( double mach, double alpha, double gamma = 1.4 ) const;
            // nested caches for C_n
            mutable std::map<double, std::map<double, std::map<double, double>>> _c_n_aCache = {};
            virtual void createC_n_aMapping( double value, double mach, double alpha, double gamma = 1.4 );
            virtual bool c_n_aExists(double mach, double alpha, double gamma = 1.4 ) const;
            virtual void clearC_n_aCache();

            //C_m_a
            // calculates cma about the components tip
            virtual double calculateC_m_a( double mach, double alpha, double gamma = 1.4 ) const = 0; // VIRTUAL
            // calculates this components C_m_a about (0,0,0)
            virtual double calculateC_m_aAtOrigin( double mach, double alpha, double gamma = 1.4 ) const;
            virtual double calculateC_m_aWithComponents( double mach, double alpha, double gamma = 1.4 ) const;
            // nested caches for C_n
            std::map<double, std::map<double, std::map<double, double>>> _c_m_aCache = {};
            virtual void createC_m_aMapping( double value, double mach, double alpha, double gamma = 1.4 );
            virtual bool c_m_aExists(double mach, double alpha, double gamma = 1.4 ) const;
            virtual void clearC_m_aCache();

            virtual Eigen::Vector3d calculateCp( double mach, double alpha, double gamma = 1.4 ) const = 0; // VIRTUAL
            virtual Eigen::Vector3d calculateCpWithComponents( double mach, double alpha, double gamma = 1.4 ) const;
            // nested caches for C_n
            std::map<double, std::map<double, std::map<double, Eigen::Vector3d>>> _cpCache = {};
            virtual void createCpMapping( Eigen::Vector3d value, double mach, double alpha, double gamma = 1.4 );
            virtual bool cpExists(double mach, double alpha, double gamma = 1.4 ) const;
            virtual void clearCpCache();

            // because omega is given about CM, this gives the moment about CM
            // calculate without omega and v for simplicity, then apply it at the end
            virtual double calculateC_m_damp(double x) const = 0; // VIRTUAL
            virtual double calculateC_m_dampWithComponents(double x) const;
            mutable std::map<double, double> _c_m_dampCache = {}; //only caching based on x
            virtual void clearC_m_dampCache();

            virtual double Cda2Cd(const double Cda, const double alpha) const;

            // friction coefficient ()
            virtual double CfNew(const double mach, const double reL) const;
            // these 2 have swapped names
            virtual double CfOrk(const double mach, const double reL) const;

            virtual double Cf(const double mach, const double reL) const { return CfOrk(mach, reL); }


            virtual double calculateCdfA(const double mach, const double reL) const = 0;
            virtual double calculateCdfAWithComponents(const double mach, const double reL) const;

            virtual double calculateCdpA(const double mach) const = 0;
            virtual double calculateCdpAWithComponents(const double mach) const;

            // constructor
            AeroComponent(
                std::unique_ptr<Shapes::AeroComponentShape> shape, std::unique_ptr<Material> material,
                std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position
                );
        public:
            // no create method as this class is abstract

            AeroComponent* parent() const override;

            // inherited from rocketInterface
            virtual Eigen::Vector3d thisWayUp() const override { return Eigen::Vector3d{-1,0,0}; }
            // need to redefine these to clarify override of interface
            // because the implementation of these functions is defined above, it needs to be specified here again so that
            // the interface knows wtf they're on about
            virtual Eigen::Vector3d cm(double time) const override { return Component::cm(time); }
            virtual Eigen::Matrix3d inertia(double time) const override { return Component::inertia(time); }
            virtual double mass(double time) const override { return Component::mass(time); }
            virtual Eigen::Vector3d thrust(double time) const override { return Component::thrust(time); }
            virtual Eigen::Vector3d thrustPosition(double time) const override { return Component::thrustPosition(time); }
            virtual double c_n( double mach, double alpha, double gamma = 1.4 ) const override;
            // calculates c_m about (0,0,0)
            virtual double c_m( double mach, double alpha, double gamma = 1.4) const override;
            virtual Eigen::Vector3d cp( double mach, double alpha, double gamma = 1.4) const override;
            virtual double c_m_damp(double time, double omega, double v) const override;

            // drag functions
            /**
             * @brief Calculates the frictional drag coefficient of this component and its children
             * 
             * @param mach mach number
             * @param reL reynolds number divided by length
             * @param alpha angle or attack in radians
             * @return double 
             * 
             * @note Re/L = density*velocity/dynamicViscocity = velocity/kinematicViscocity
             */
            //virtual double Cdf(const double mach, const double reL, const double alpha) const override { return CdfA(mach, reL); }
            virtual double Cdf(const double mach, const double reL, const double alpha) const override { return Cda2Cd(CdfA(mach, reL), alpha); }

            /**
             * @brief Calculates the axial frictional drag coefficient of this component and its children
             * 
             * @param mach mach number
             * @param reL reynolds number divided by length
             * @return double 
             * 
             * @note Re/L = density*velocity/dynamicViscocity = velocity/kinematicViscocity
             */
            virtual double CdfA(const double mach, const double reL) const;

            virtual double Cdp(const double mach, const double alpha) const override { return Cda2Cd(CdpA(mach), alpha); }

            virtual double CdpA(const double mach) const;

            virtual double Cdb(const double mach, const double time, const double alpha) const override { return Cda2Cd(CdbA(mach, time), alpha); }
            virtual double CdbA(const double mach, const double time) const { return 0; }
            virtual double Cdotb(const double mach) const;

            // shape stuff
            // these are defined by rocket interface
            virtual double referenceArea() const override;
            virtual double referenceLength() const override = 0;


            virtual double wettedArea() const; // surface area of the shape exposed to the air
            virtual double planformArea() const;
            virtual Eigen::Vector3d planformCenter() const; // geometric center of the shapes planform
            // reference length is undefined in this context and remains virtual

            //new
            virtual double c_n_a( double mach, double alpha, double gamma = 1.4 ) const;
            virtual double c_m_a( double mach, double alpha, double gamma = 1.4) const;
            virtual std::vector< std::shared_ptr<AeroComponent> > aeroComponents() const = 0; // VIRTUAL

            // returns the diameter of the body at a given distance from the nosecone
            virtual double bodyRadius(double x) const = 0; // VIRTUAL

            // getter and setter for shape
            virtual Shapes::AeroComponentShape* shape() const;
            virtual void setShape( std::unique_ptr<Shapes::AeroComponentShape> shape );
            //virtual void setShape( Shapes::AeroComponentShape* shape ); // CLEAR CACHES
            // getter and setter for finish
            virtual Finish* finish() const;
            virtual void setFinish( std::unique_ptr<Finish> finish ); // CLEAR CACHES
            // getter and setter for material
            virtual Material* material() const;
            virtual void setMaterial( std::unique_ptr<Material> material ); // CLEAR CACHES
            
            virtual double calculateLowestPoint() const = 0;
            virtual double lowestPoint() const;

            virtual double calculateSurfaceDistanceTravelled(double x) const = 0;
            virtual double maxSurfaceDistanceTravelled() const = 0;
            virtual double surfaceDistanceTravelled(double x) const;

            virtual double finenessRatio() const = 0;
            virtual double finenessRatioRocket() const;
    };

}

#endif