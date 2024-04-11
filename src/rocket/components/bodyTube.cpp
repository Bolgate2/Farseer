#include "bodyTube.hpp"

namespace Rocket{

BodyTube::BodyTube(std::string name, Eigen::Vector3d position, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish):
Component(name, position)
{
    setMaterial(std::move(material));
    setFinish(std::move(finish));
}


BodyTube::BodyTube(double height, double diameter, double thickness, bool filled, std::string name, Eigen::Vector3d position, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish):
Component(name, position)
{
    setHeight(height);
    setDiameter(diameter);
    setThickness(thickness);
    setFilled(filled);
    setMaterial(std::move(material));
    setFinish(std::move(finish));
}


json BodyTube::propertiesToJson() {
    return json {
        {"height", getHeight()},
        {"diameter", getDiameter()},
        {"thickness", getThickness()},
        {"filled", getFilled()},
        {"material", getMaterial()->toJson()},
        {"finish", getFinish()->toJson()}
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
    json desiredFinish = properties.at("finish");
    setHeight(desiredHeight);
    setDiameter(desiredDiameter);
    setThickness(desiredThickness);
    setFilled(desiredFilled);
    setMaterial(std::move(
        Material::fromJson(desiredMaterial)
    ));
    setFinish(std::move(
        Finish::fromJson(desiredFinish)
    ));
}

}