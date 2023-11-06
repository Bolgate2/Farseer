#ifndef FIN_H_
#define FIN_H_

#include "../externalComponent.hpp"
#include "../../shapes/components/fins/fin.hpp"
#include "finSet.hpp"
#include "nanValues.hpp"
#include <memory>

namespace Rocket{
    class FinSet;

    class Fin: public ExternalComponent{
        private:
            static std::string defaultName;
            std::unique_ptr<Shapes::FinComponentShape> _shape;
        protected:
            // overriding cm to be in local coordinates, with the origin being the top corner of the fin in this orientation -> D
            // the fin set will handle the global coords
            // this affects the inertia, it will now be reported about the origin here
            Eigen::Vector3d calculateCm(double time) const override;
            Eigen::Matrix3d calculateInertia(double time) const override;
            
            // using caching for this
            virtual void clearCaches() override;
            Eigen::Array<double, 6, 1> _cpInterpPolyCoeffs = Eigen::Array<double, 6, 1>::Ones()*NAN_D;
            virtual Eigen::Array<double, 6, 1> cpInterpPolyCoeffs() const;

            std::weak_ptr<FinSet> _finSet;
            // aero component functions
            virtual double calculateC_m_damp(double x, double omega, double v) const override { return 0; } // this is calculated at the fin set
            virtual double calculateC_m_a( double mach, double alpha, double gamma = 1.4 ) const override { return 0; } // formula not given

            virtual double subsonicCNa( double mach, double alpha ) const;
            virtual double supersonicCNa( double mach, double alpha, double gamma = 1.4 ) const;
            virtual double calculateC_n_a( double mach, double alpha, double gamma = 1.4 ) const override;
            virtual Eigen::Vector3d calculateCp( double mach, double alpha, double gamma = 1.4 ) const override;

        public:
            // constructor
            Fin(std::unique_ptr<Shapes::FinComponentShape> shape, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, std::string name);
            virtual FinSet* finSet() const;
            virtual void setFinSet( FinSet* finSet );
            // aero component functions
            virtual Shapes::FinComponentShape* shape() const override;
            virtual void setShape( std::unique_ptr<Shapes::AeroComponentShape> shape ) override;
            virtual void setShape( std::unique_ptr<Shapes::ExternalComponentShape> shape ) override;
            virtual void setShape( std::unique_ptr<Shapes::FinComponentShape> shape ); // RECALCULATE CP INTERP POLYS
            virtual double referenceArea() const override; // using reference area of the parent
            virtual double referenceLength() const override; // using reference length of the parent

            // tree stuff. fins dont exist in the component tree as they are a part of their fin set
            virtual std::vector< std::shared_ptr<AeroComponent> > aeroComponents() const override { return std::vector< std::shared_ptr<AeroComponent> >{}; };
            virtual std::vector<std::shared_ptr<Component>> components() const override { return std::vector< std::shared_ptr<Component> >{}; };
            virtual AeroComponent* parent() const override { return nullptr; };
            virtual void setParent( Component* parent ) override { /*do nothing*/ };
            virtual void addComponent(Component* component) override { /*do nothing*/ };
            virtual void removeComponent(Component* component) override { /*do nothing*/ };
            // positions stuff. the fins position is dictated by its set
            virtual Eigen::Vector3d position() const override;
            virtual void setPosition() { /*do nothing*/ }

            //fin stuff
            virtual double chord(double y) const { return shape()->chord(y); } // returns the chord at the given point y from the root chord
            virtual double mac() const { return shape()->mac(); } // mean area chord
            virtual double xMacLeadingEdge() const { return shape()->xMacLeadingEdge(); }
            virtual double yMac() const { return shape()->yMac(); }
            virtual double midChordSweep() const { return shape()->midChordSweep(); }
            virtual double yMax() const { return shape()->yMax(); } // distance to the outermost point of the fin from the root. used for aspect ratio

            
    };
}

#endif