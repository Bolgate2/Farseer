# ifndef BODY_TUBE_H_
# define BODY_TUBE_H_

#include "../bodyComponent.hpp"
#include "../../shapes/components/bodyTube/bodyTube.hpp"

namespace Rocket{
    class BodyTube: public BodyComponent{
        private:
            static std::string defaultName;
            std::unique_ptr<Shapes::BodyTubeComponentShape> _shape;
        protected:
            // aero functions
            virtual double calculateCdpA(const double mach) const override {return 0;} // no pressure drag from components

            // constructors
            BodyTube(
                double radius, double length, double thickness, std::unique_ptr<Material> material,
                std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position
                );
            
            // constructor with no thickness
            BodyTube(
                double radius, double length, std::unique_ptr<Material> material,
                std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position
                );
            
            // constructor with existing shape
            BodyTube(
                std::unique_ptr<Shapes::BodyTubeComponentShape> shape, std::unique_ptr<Material> material,
                std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position
                );
        public:
            // create methods
            [[nodiscard]]
            static std::shared_ptr<BodyTube> create(
                double radius, double length, double thickness, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish,
                Stage* parent = nullptr, std::string name = BodyTube::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero()
                );
            
            // create with no thickness
            [[nodiscard]]
            static std::shared_ptr<BodyTube> create(
                double radius, double length, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish,
                Stage* parent = nullptr, std::string name = BodyTube::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero()
                );
            
            // create with existing shape
            [[nodiscard]]
            static std::shared_ptr<BodyTube> create(
                std::unique_ptr<Shapes::BodyTubeComponentShape> shape, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish,
                Stage* parent = nullptr, std::string name = BodyTube::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero()
                );
            
            // redefine shape getter bc shape has changed
            virtual Shapes::BodyTubeComponentShape* shape() const override;
            virtual void setShape( std::unique_ptr<Shapes::AeroComponentShape> shape ) override;
            virtual void setShape( std::unique_ptr<Shapes::BodyComponentShape> shape ) override;
            virtual void setShape( std::unique_ptr<Shapes::BodyTubeComponentShape> shape );

            // surf distance
            virtual double maxSurfaceDistanceTravelled() const override;
            virtual double calculateSurfaceDistanceTravelled(double x) const override;
    };
}

# endif