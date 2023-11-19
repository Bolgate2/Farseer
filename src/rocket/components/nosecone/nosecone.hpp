#ifndef NOSECONE_H_
#define NOSECONE_H_

#include "../bodyComponent.hpp"
#include "../../shapes/components/nosecone/nosecone.hpp"
#include "noseconeData.hpp"
#include <tuple>
#include <map>
#include <unordered_map>

namespace Rocket{

    class Nosecone: public BodyComponent{
        private:
            static std::string defaultName;
            std::unique_ptr<Shapes::NoseconeComponentShape> _shape;

        protected:
            virtual double cdm0() const;
            virtual double cDotStag(const double mach) const;
            virtual double conicalSubsonicCdpdot(const double mach) const;
            virtual double conicalTransonicCdpdot(const double mach) const;
            virtual double conicalSupersonicCdpdot(const double mach) const;
            virtual double conicalCdpdot(const double mach) const;
            
            virtual double finenessCorrection(const double mach, const double c3 ) const;
            virtual double ellipsoidCdpdot(const double mach) const;
            virtual double powerCdpdot(const double mach) const;
            virtual double parabolicCdpdot(const double mach) const;
            virtual double haackCdpdot(const double mach) const;

            // aero functions
            virtual double calculateCdpA(const double mach) const override;
            // overriding inertia
            Eigen::Matrix3d calculateInertia(double time) const override;
            // constructors
            Nosecone(Shapes::NoseconeShapeTypes type, double radius, double length, double thickness, double shapeParam, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position);
            Nosecone(Shapes::NoseconeShapeTypes type, double radius, double length, double shapeParam, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position);
            Nosecone(std::unique_ptr<Shapes::NoseconeComponentShape> shape, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position);
        public:
            [[nodiscard]]
            static std::shared_ptr<Nosecone> create(
                Shapes::NoseconeShapeTypes type, double radius, double length, double thickness, double shapeParam, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish,
                Stage* parent = nullptr, std::string name = Nosecone::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero()
                );
            /*
            // create with no thickness
            [[nodiscard]]
            static std::shared_ptr<Nosecone> create(
                Shapes::NoseconeShapeTypes type, double radius, double length, double shapeParam, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish,
                BodyComponent* parent = nullptr, std::string name = Nosecone::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero()
                );
            // create with existing shape
            [[nodiscard]]
            static std::shared_ptr<Nosecone> create(
                std::unique_ptr<Shapes::NoseconeComponentShape> shape, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish,
                BodyComponent* parent = nullptr, std::string name = Nosecone::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero()
                );
            */
            Shapes::NoseconeComponentShape* shape() const override;
            virtual void setShape( std::unique_ptr<Shapes::AeroComponentShape> shape ) override;
            virtual void setShape( std::unique_ptr<Shapes::BodyComponentShape> shape ) override;
            virtual void setShape( std::unique_ptr<Shapes::NoseconeComponentShape> shape );

            virtual double shapeParam() const;
            virtual double zeta() const;
            virtual void setShapeParam(double val); // CLEAR CACHES

            virtual Shapes::NoseconeShapeTypes type() const { return shape()->type(); }

            virtual double maxSurfaceDistanceTravelled() const override;
            virtual double calculateSurfaceDistanceTravelled(double x) const override;
    };
}

#endif