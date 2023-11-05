#include "nosecone.hpp"
#include "../../shapes/components/nosecone/nosecone.hpp"
#include "maths.hpp"
#include <iostream>
#include <memory>
#include <cmath>

namespace Rocket{

    std::string Nosecone::defaultName = "Nosecone";
    // constructors
    Nosecone::Nosecone(Shapes::NoseconeShapeTypes type, double radius, double length, double thickness, double shapeParam, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position):
    BodyComponent(nullptr, std::move(material), std::move(finish), name, position )
    {
        auto shape = std::make_unique<Shapes::NoseconeComponentShape>(type, radius, length, thickness, shapeParam);
        setShape(std::move(shape));
    }
    // constructor without thickness
    Nosecone::Nosecone(Shapes::NoseconeShapeTypes type, double radius, double length, double shapeParam, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position):
    BodyComponent(nullptr, std::move(material), std::move(finish), name, position )
    {
        std::cerr << "NOSECONE WITHOUT THICKNESS HAS NOT YET BEEN IMPLEMENTED. Using thickness 0.001" << std::endl;
        auto shape = std::make_unique<Shapes::NoseconeComponentShape>(type, radius, length, 0.001, shapeParam);
        setShape(std::move(shape));
    }
    // constructor with existing shape
    Nosecone::Nosecone(std::unique_ptr<Shapes::NoseconeComponentShape> shape, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position):
    BodyComponent(nullptr, std::move(material), std::move(finish), name, position )
    {
        setShape(std::move(shape));
    }

    // creator
    std::shared_ptr<Nosecone> Nosecone::create(Shapes::NoseconeShapeTypes type, double radius, double length, double thickness, double shapeParam, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, BodyComponent *parent, std::string name, Eigen::Vector3d position)
    {
        auto obj = std::shared_ptr<Nosecone>(
            new Nosecone(type, radius, length, thickness, shapeParam, std::move(material), std::move(finish), name, position)
        );
        if(parent != nullptr){
            parent->addComponent(obj.get());
        }
        return obj;
    }

    // GETTERS AND SETTERS FOR SHAPE
    Shapes::NoseconeComponentShape* Nosecone::shape() const {
        return _shape.get();
    }

    void Nosecone::setShape( std::unique_ptr<Shapes::AeroComponentShape> shape ){
        // try to cast the underlying pointer to a pointer of the inherited class
        auto castedShapePointer = dynamic_cast<Shapes::NoseconeComponentShape*>(shape.get());

        if(castedShapePointer != nullptr){
            // if casting is successful the a unique pointer of the correct type is crated and the old one is released
            auto newShapeUniquePtr = std::unique_ptr<Shapes::NoseconeComponentShape>(castedShapePointer);
            shape.release();
            setShape(std::move(newShapeUniquePtr));
        }
        // if the function has reached here it has failed
        std::cerr << "Invalid shape for nosecone\n"; // TODO: make this more descriptive
    }

    
    void Nosecone::setShape( std::unique_ptr<Shapes::BodyComponentShape> shape ){
        // try to cast the underlying pointer to a pointer of the inherited class
        auto castedShapePointer = dynamic_cast<Shapes::NoseconeComponentShape*>(shape.get());
        if(castedShapePointer != nullptr){
            // if casting is successful the a unique pointer of the correct type is crated and the old one is released
            auto newShapeUniquePtr = std::unique_ptr<Shapes::NoseconeComponentShape>(castedShapePointer);
            shape.release();
            setShape(std::move(newShapeUniquePtr));
        }
        // if the function has reached here it has failed
        std::cerr << "Invalid shape for nosecone\n"; // TODO: make this more descriptive
    }

    void Nosecone::setShape(std::unique_ptr<Shapes::NoseconeComponentShape> shape){
        // try to cast the underlying pointer to a pointer of the inherited class
        clearCaches();
        _shape = std::move(shape);
    }

    // GETTER AND SETTER FOR SHAPEPARAM
    double Nosecone::shapeParam() const {
        return shape()->shapeParam();
    }

    void Nosecone::setShapeParam(double val){
        clearCaches();
        shape()->setShapeParam(val);
    }

    Eigen::Matrix3d Nosecone::calculateInertia(double time) const {
        // inertia about cm
        auto thisInertia = shape()->inertia(); // inertia/density of the shape about cm
        auto disp = -position(); // THIS LINE HAS CHANGED, the nosecones inertia is calculated about its tip
        auto thisVolume = shape()->volume();
        auto zeroInertia = Utils::parallel_axis_transform(thisInertia, disp, thisVolume);
        zeroInertia *= material()->density; // multiplying by density so it's the actual inertia
        return zeroInertia;
    }
}