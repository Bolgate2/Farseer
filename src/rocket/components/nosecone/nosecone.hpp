#ifndef NOSECONE_H_
#define NOSECONE_H_

#include "../bodyComponent.hpp"
#include "../../shapes/components/nosecone/nosecone.hpp"

namespace Rocket{
    class Nosecone: public BodyComponent{
        private:
            static std::string defaultName;
            std::unique_ptr<Shapes::NoseconeComponentShape> _shape;
        protected:
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
            virtual void setShapeParam(double val); // CLEAR CACHES


    };
}

#endif