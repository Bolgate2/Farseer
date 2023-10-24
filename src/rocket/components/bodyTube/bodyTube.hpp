# ifndef BODY_TUBE_H_
# define BODY_TUBE_H_

#include "../bodyComponent.hpp"
#include "../../shapes/bodyTube/bodyTube.hpp"

namespace Rocket{
    class BodyTube: public BodyComponent{
        private:
            std::string _defaultName = "Body Tube";
            Shapes::BodyTubeShape* _shape;
            double calculateC_n_a( double mach, double alpha, double gamma = 1.4 );
            double calculateC_m_a( double mach, double alpha, double gamma = 1.4 );
            Eigen::Vector3d calculateCp( double mach, double alpha, double gamma = 1.4 );
            double calculateC_m_damp( double time, double omega, double v );
        public:
            BodyTube(double radius, double length, Material* material, Finish* finish,
                BodyComponent* parent = NULL, std::string name = BodyComponent::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero());
            BodyTube(double radius, double length, double thickness, Material* material, Finish* finish,
                BodyComponent* parent = NULL, std::string name = BodyComponent::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero());
            BodyTube(Shapes::BodyTubeShape* shape, Material* material, Finish* finish,
                BodyComponent* parent = NULL, std::string name = BodyComponent::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero());
            // redefine shape getter bc shape has changed
            virtual Shapes::BodyTubeShape* shape();
            virtual void setShape( Shapes::AeroShape* shape );
            virtual double calculateMass(double time);
    };
}

# endif