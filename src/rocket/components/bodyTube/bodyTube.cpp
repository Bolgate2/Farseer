#include "bodyTube.hpp"
#include "../../shapes/components/bodyTube/bodyTube.hpp"

#include <cmath>

namespace Rocket{

    std::string BodyTube::defaultName = "Body Tube";
    // constructors
    BodyTube::BodyTube(double radius, double length, double thickness, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position):
    BodyComponent(nullptr, std::move(material), std::move(finish), name, position )
    {
        auto shp = std::make_unique<Shapes::BodyTubeComponentShape>(radius, length, thickness);
        setShape(std::move(shp));
    }

    // constructor without thickness (for a filled tube)
    BodyTube::BodyTube(double radius, double length, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position):
    BodyComponent(nullptr, std::move(material), std::move(finish), name, position)
    {
        auto shp = std::make_unique<Shapes::BodyTubeComponentShape>(radius, length);
        setShape(std::move(shp));
    }

    // constructor with a preconstructed shape
    BodyTube::BodyTube(std::unique_ptr<Shapes::BodyTubeComponentShape> shape, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position):
    BodyComponent(nullptr, std::move(material), std::move(finish), name, position)
    {
        setShape(std::move(shape));
    }

    // creators
    std::shared_ptr<BodyTube> BodyTube::create(double radius, double length, double thickness, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, BodyComponent *parent, std::string name, Eigen::Vector3d position)
    {
        auto obj = std::shared_ptr<BodyTube>(
            new BodyTube(radius, length, thickness, std::move(material), std::move(finish), name, position)
        );
        if(parent != nullptr){
            parent->addComponent(obj.get());
        }
        return obj;
    }
    // creator without thickness
    std::shared_ptr<BodyTube> BodyTube::create(double radius, double length, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, BodyComponent *parent, std::string name, Eigen::Vector3d position)
    {
        auto obj = std::shared_ptr<BodyTube>(
            new BodyTube(radius, length, std::move(material), std::move(finish), name, position)
        );
        if(parent != nullptr){
            parent->addComponent(obj.get());
        }
        return obj;
    }
    // creator with existing shape
    std::shared_ptr<BodyTube> BodyTube::create(std::unique_ptr<Shapes::BodyTubeComponentShape> shape, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, BodyComponent *parent, std::string name, Eigen::Vector3d position)
    {
        auto obj = std::shared_ptr<BodyTube>(
            new BodyTube(std::move(shape), std::move(material), std::move(finish), name, position)
        );
        if(parent != nullptr){
            parent->addComponent(obj.get());
        }
        return obj;
    }

    void BodyTube::setShape( std::unique_ptr<Shapes::AeroComponentShape> shape ){
        // try to cast the underlying pointer to a pointer of the inherited class
        auto castedShapePointer = dynamic_cast<Shapes::BodyTubeComponentShape*>(shape.get());

        if(castedShapePointer != nullptr){
            // if casting is successful the a unique pointer of the correct type is crated and the old one is released
            auto newShapeUniquePtr = std::unique_ptr<Shapes::BodyTubeComponentShape>(castedShapePointer);
            shape.release();
            setShape(std::move(newShapeUniquePtr));
        }
        // if the function has reached here it has failed
        std::cerr << "Invalid shape for body tube\n"; // TODO: make this more descriptive
    }

    void BodyTube::setShape( std::unique_ptr<Shapes::BodyComponentShape> shape ){
        // try to cast the underlying pointer to a pointer of the inherited class
        auto castedShapePointer = dynamic_cast<Shapes::BodyTubeComponentShape*>(shape.get());
        if(castedShapePointer != nullptr){
            // if casting is successful the a unique pointer of the correct type is crated and the old one is released
            auto newShapeUniquePtr = std::unique_ptr<Shapes::BodyTubeComponentShape>(castedShapePointer);
            shape.release();
            setShape(std::move(newShapeUniquePtr));
        }
        // if the function has reached here it has failed
        std::cerr << "Invalid shape for body tube\n"; // TODO: make this more descriptive
    }

    void BodyTube::setShape(std::unique_ptr<Shapes::BodyTubeComponentShape> shape){
        // try to cast the underlying pointer to a pointer of the inherited class
        _shape = std::move(shape);
        clearCaches();
    }

    Shapes::BodyTubeComponentShape* BodyTube::shape() const {
        return _shape.get();
    }
    
}