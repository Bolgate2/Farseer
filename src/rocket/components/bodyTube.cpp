#include "bodyTube.hpp"

namespace Rocket{

BodyTube::BodyTube(std::string name, Eigen::Vector3d position)
{}


BodyTube::BodyTube(double height, double diameter, double thickness, bool filled, std::string name, Eigen::Vector3d position)
{
    _height = height;
    _diameter = diameter;
    _thickness = thickness;
    _filled = filled;
}


json BodyTube::propertiesToJson() {
    return json {
        {"height", getHeight()},
        {"diameter", getDiameter()},
        {"thickness", getThickness()},
        {"filled", getFilled()}
    };
}

// this will also go about creating sub-components
void BodyTube::jsonToProperties(json j) {
    json properties = j.at("properties");
    double desiredHeight = properties.at("height");
    double desiredDiameter = properties.at("diameter");
    double desiredThickness = properties.at("thickness");
    bool desiredFilled = properties.at("filled");
    setHeight(desiredHeight);
    setDiameter(desiredDiameter);
    setThickness(desiredThickness);
    setFilled(desiredFilled);
}

}