#include "bodyTube.hpp"
#include "../../shapes/bodyTube/bodyTube.hpp"

#include <cmath>

namespace Rocket{

    std::string BodyTube::defaultName = "Body Tube";
    // constructors
    BodyTube::BodyTube(double radius, double length, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, BodyComponent* parent, std::string name, Eigen::Vector3d position):
    BodyComponent( std::move(material), std::move(finish), parent, name, position )
    {
        auto shp = std::make_unique<Shapes::BodyTubeShape>(radius, length);
        //auto s = Shapes::BodyTubeShape(radius, length);
        setShape(std::move(shp));
    }

    BodyTube::BodyTube(double radius, double length, double thickness, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, BodyComponent* parent, std::string name, Eigen::Vector3d position):
    BodyComponent( std::move(material), std::move(finish), parent, name, position )
    {
        auto shp = std::make_unique<Shapes::BodyTubeShape>(radius, length, thickness);
        setShape(std::move(shp));
    }

    BodyTube::BodyTube(std::unique_ptr<Shapes::BodyTubeShape> shape, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, BodyComponent* parent, std::string name, Eigen::Vector3d position):
    BodyComponent(std::move(material), std::move(finish), parent, name, position)
    {
        setShape(std::move(shape));
    }

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
    
    Shapes::BodyTubeShape* BodyTube::shape() {
        return _shape.get();
    }

    void BodyTube::setShape( std::unique_ptr<Shapes::AeroShape> shape ){
        // try to cast the underlying pointer to a pointer of the inherited class
        auto castedShapePointer = dynamic_cast<Shapes::BodyTubeShape*>(shape.get());

        if(castedShapePointer != NULL){
            // if casting is successful the a unique pointer of the correct type is crated and the old one is released
            auto newShapeUniquePtr = std::unique_ptr<Shapes::BodyTubeShape>(castedShapePointer);
            shape.release();
            setShape(std::move(newShapeUniquePtr));
        }
        // if the function has reached here it has failed
        std::cerr << "Invalid shape for body tube\n"; // TODO: make this more descriptive
    }

    
    void BodyTube::setShape( std::unique_ptr<Shapes::BodyComponentShape> shape ){
        // try to cast the underlying pointer to a pointer of the inherited class
        auto castedShapePointer = dynamic_cast<Shapes::BodyTubeShape*>(shape.get());

        if(castedShapePointer != NULL){
            // if casting is successful the a unique pointer of the correct type is crated and the old one is released
            auto newShapeUniquePtr = std::unique_ptr<Shapes::BodyTubeShape>(castedShapePointer);
            shape.release();
            setShape(std::move(newShapeUniquePtr));
        }
        // if the function has reached here it has failed
        std::cerr << "Invalid shape for body tube\n"; // TODO: make this more descriptive
    }

    void BodyTube::setShape(std::unique_ptr<Shapes::BodyTubeShape> shape){
        // try to cast the underlying pointer to a pointer of the inherited class
        _shape = std::move(shape);
    }
    
}