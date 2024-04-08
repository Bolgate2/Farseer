#pragma once
#include "component.hpp"

namespace Rocket{

class BodyTube : public Component{
    private:
        double _height = 0;
        double _diameter = 0;
        double _thickness = 0;
        bool _filled = false;
    
    protected:
        virtual json propertiesToJson() override;
        // this will also go about creating sub-components
        virtual void jsonToProperties(json j) override;
    
    public:
        BodyTube(std::string name = "", Eigen::Vector3d position = Eigen::Vector3d::Zero());
        BodyTube(double height, double diameter, double thickness, bool filled = false, std::string name = "", Eigen::Vector3d position = Eigen::Vector3d::Zero());

        double getHeight(){ return _height; };
        void setHeight(double height) { _height = std::max(0.0,height); };
        double getDiameter(){ return _diameter; };
        void setDiameter(double diameter) { _diameter = std::max(0.0,diameter); };
        double getThickness(){ return _thickness; };
        void setThickness(double thickness) { _thickness = std::max(0.0,thickness); };
        bool getFilled(){ return _filled; };
        void setFilled(bool filled) { _filled = filled; };

        virtual std::string type() override { return COMPONENT_NAMES::BODY_TUBE; };
        virtual std::vector<std::string> allowedComponents() override {
            return std::vector<std::string>{
                COMPONENT_NAMES::MOTOR,
                COMPONENT_NAMES::FIN_SET
            };
        };
};

}