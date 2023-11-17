#ifndef FIN_SET_H_
#define FIN_SET_H_

#include "../externalComponent.hpp"
#include "fin.hpp"

namespace Rocket{
    class Fin;

    class FinSet : public ExternalComponent{
        private:
            static std::string defaultName;
            std::unique_ptr<Fin> _fin;
            int _numFins;
            std::vector<Eigen::Matrix3d> _finRotations = {};
            std::vector<Eigen::Matrix3d> finRotations() const;
            // TODO: fin cross section
            // TODO: fin cant
            // TODO: fin root fillets
        protected:

            // cache clearing
            virtual void clearCaches() override;
            // component functions
            virtual double calculateMass(double time) const override;
            virtual Eigen::Matrix3d calculateInertia(double time) const override;
            virtual Eigen::Vector3d calculateCm(double time) const override;

            // aero functions
            virtual double calculateC_n_a( double mach, double alpha, double gamma = 1.4 ) const override;
            virtual double calculateC_m_a( double mach, double alpha, double gamma = 1.4 ) const override;
            virtual Eigen::Vector3d calculateCp( double mach, double alpha, double gamma = 1.4 ) const override;
            virtual double calculateC_m_damp(double x) const override;

            virtual double calculateCdfA(const double mach, const double reL) const override;

            // constructor
            FinSet(int numFins, std::string name = FinSet::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero() );
        public:
            // creator
            [[nodiscard]]
            static std::shared_ptr<FinSet> create(
                std::unique_ptr<Fin> fin, int numFins, AeroComponent* parent, std::string name = FinSet::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero()
                );

            // component stuff
            virtual Finish* finish() const override;
            virtual void setFinish( std::unique_ptr<Finish> finish ) override; // CLEAR CACHES
            // getter and setter for material
            virtual Material* material() const override;
            virtual void setMaterial( std::unique_ptr<Material> material ) override; // CLEAR CACHES

            // aero functions
            virtual double referenceArea() const override { return parent()->referenceArea(); };
            virtual double referenceLength() const override { return parent()->referenceLength(); };
            virtual double wettedArea() const override; // surface area of the shape exposed to the air
            
            // new functions
            virtual Fin* fin() const;
            virtual void setFin( std::unique_ptr<Fin> fin );
            virtual int numFins() const;
            virtual void setNumFins( int num );

            virtual double calculateLowestPoint() const override;
            
            virtual double maxSurfaceDistanceTravelled() const override;
            virtual double calculateSurfaceDistanceTravelled(double x) const override;


    };
}


#endif