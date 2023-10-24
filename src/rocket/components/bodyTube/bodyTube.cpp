#include "bodyTube.hpp"
#include "../../shapes/bodyTube/bodyTube.hpp"

#include <cmath>

namespace Rocket{
    // constructors
    BodyTube::BodyTube(double radius, double length, Material* material, Finish* finish,BodyComponent* parent, std::string name, Eigen::Vector3d position):
    BodyComponent( material, finish, parent, name, position )
    {
        std::cout << "hi from bt constructor\n";
        auto s = Shapes::BodyTubeShape(radius, length);
        setShape(&s);
    }

    BodyTube::BodyTube(double radius, double length, double thickness, Material* material, Finish* finish,BodyComponent* parent, std::string name, Eigen::Vector3d position):
    BodyComponent( material, finish, parent, name, position )
    { 
        std::cout << "hi from bt constructor\n";
        auto s = Shapes::BodyTubeShape(radius, length, thickness);
        setShape(&s);
    }

    BodyTube::BodyTube(Shapes::BodyTubeShape* shape, Material* material, Finish* finish, BodyComponent* parent, std::string name, Eigen::Vector3d position):
    BodyComponent(shape, material, finish, parent, name, position)
    {}

    // aero stuff
    double BodyTube::calculateC_n_a( double mach, double alpha, double gamma){
        // the body tubes only normal force contribution is from normal force
        auto bodyLiftCorrection = bodyLift( alpha );
        return bodyLiftCorrection;
    }

    double BodyTube::calculateC_m_a( double mach, double alpha, double gamma){
        return 0;
    }

    Eigen::Vector3d BodyTube::calculateCp( double mach, double alpha, double gamma){
        return bodyLiftCp();
    }

    double BodyTube::calculateC_m_damp( double time, double omega, double v ){
        auto totalCm = root()->cm(time);
        auto cmX = totalCm.x();
        auto compTop = position().x(); // smaller, closer to top
        auto compBottom = compTop + length(); // bigger, further from top
        // calculated seperately for top and bottom
        if( compBottom <= cmX | compTop >= cmX ){
            return c_m_damp_Func(length(), radius(), omega, v);
        }
        auto topLen = cmX - compTop;
        auto bottomLen = length() - topLen;
        auto topCoeff = c_m_damp_Func(topLen, radius(), omega, v);
        auto bottomCoeff = c_m_damp_Func(bottomLen, radius(), omega, v);
        return topCoeff + bottomCoeff;
    }
    
    Shapes::BodyTubeShape* BodyTube::shape(){
        return _shape;
    }
    
    void BodyTube::setShape( Shapes::AeroShape* shape ){
        auto castedShape = dynamic_cast<Shapes::BodyTubeShape*>(shape);
        if(castedShape != NULL){
            _shape = castedShape;
            return;
        }
        // if the function has reached here it has failed
        std::cerr << "Invalid shape for body tube\n"; // TODO: make this more descriptive
    }

    double BodyTube::calculateMass(double time){
        auto density = this->material()->density;
        auto vol = shape()->volume();
        return density * vol;
    }

}