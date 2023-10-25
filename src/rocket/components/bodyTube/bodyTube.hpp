# ifndef BODY_TUBE_H_
# define BODY_TUBE_H_

#include "../bodyComponent.hpp"
#include "../../shapes/bodyTube/bodyTube.hpp"

namespace Rocket{
    class BodyTube: public BodyComponent{
        private:
            static std::string defaultName;
            std::unique_ptr<Shapes::BodyTubeShape> _shape;
            double calculateC_n_a( double mach, double alpha, double gamma = 1.4 );
            double calculateC_m_a( double mach, double alpha, double gamma = 1.4 );
            Eigen::Vector3d calculateCp( double mach, double alpha, double gamma = 1.4 );
            double calculateC_m_damp( double time, double omega, double v );
        public:
            BodyTube(double radius, double length, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish,
                BodyComponent* parent = NULL, std::string name = BodyTube::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero());
            BodyTube(double radius, double length, double thickness, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish,
                BodyComponent* parent = NULL, std::string name = BodyTube::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero());
            BodyTube(std::unique_ptr<Shapes::BodyTubeShape> shape, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish,
                BodyComponent* parent = NULL, std::string name = BodyTube::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero());
            // redefine shape getter bc shape has changed
            virtual Shapes::BodyTubeShape* shape();
            virtual void setShape( std::unique_ptr<Shapes::AeroShape> shape ) override;
            virtual void setShape( std::unique_ptr<Shapes::BodyComponentShape> shape ) override;
            virtual void setShape( std::unique_ptr<Shapes::BodyTubeShape> shape );
    };
}

# endif