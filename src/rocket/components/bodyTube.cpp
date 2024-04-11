#include "bodyTube.hpp"

namespace Rocket{

BodyTube::BodyTube(std::string name, Eigen::Vector3d position, std::unique_ptr<Material> material):
Component(name, position)
{
    setMaterial(std::move(material));
}


BodyTube::BodyTube(double height, double diameter, double thickness, bool filled, std::string name, Eigen::Vector3d position, std::unique_ptr<Material> material):
Component(name, position)
{
    _height = height;
    _diameter = diameter;
    _thickness = thickness;
    _filled = filled;
    setMaterial(std::move(material));
}


json BodyTube::propertiesToJson() {
    return json {
        {"height", getHeight()},
        {"diameter", getDiameter()},
        {"thickness", getThickness()},
        {"filled", getFilled()},
        {"material", getMaterial()->toJson()}
    };
}

// this will also go about creating sub-components
void BodyTube::jsonToProperties(json j) {
    json properties = j.at("properties");
    double desiredHeight = properties.at("height");
    double desiredDiameter = properties.at("diameter");
    double desiredThickness = properties.at("thickness");
    bool desiredFilled = properties.at("filled");
    json desiredMaterial = properties.at("material");
    setHeight(desiredHeight);
    setDiameter(desiredDiameter);
    setThickness(desiredThickness);
    setFilled(desiredFilled);
    std::unique_ptr<Material> mat = std::move( Material::fromJson(desiredMaterial) );
    setMaterial(std::move(
        Material::fromJson(desiredMaterial)
    ));
}

}